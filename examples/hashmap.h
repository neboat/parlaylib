template <class K, class V>
struct hashmap {
private:
  using KV = std::pair<K,V>;
  using index = unsigned long;
  using ptr = size_t;
  long m;
  index first_index(K k) { return k.hash() % m;}
  index next_index(index h) { return (h + 1 == m) ? 0 : h + 1; }
  static const ptr empty_flag = (1ul << 63);
  parlay::sequence<std::atomic<ptr>> H;
  KV* deref(ptr p) {return (KV*) (p & ~empty_flag);}
  bool is_deleted(ptr p) {return (bool) (p & empty_flag);}
  ptr gen_ptr(KV* p, bool deleted=false) {
    return (size_t) p | (((size_t) deleted) << 63);
  }
  //parlay::type_allocator<KV> kv_alloc;
  
public:
  hashmap(long size) 
    : m(100 + static_cast<index>(1.5 * size)),
      H(parlay::sequence<std::atomic<ptr>>::from_function(m, [&] (size_t i) {
	    return 0ul;})) {}

  ~hashmap() {
    parlay::parallel_for(0,m, [&] (long i) {
	ptr kv = H[i].load();
	if (kv != 0) {
	  deref(kv)->~KV();
	  parlay::p_free(deref(kv));
	}},1000);
  }

  bool insert(const K& k, const V& v) {
    //auto kv = kv_alloc.alloc();
    auto kv = (ptr) parlay::p_malloc(sizeof(KV)); //kv_alloc.alloc();
    new ((KV*) kv) KV{std::move(k), std::move(v)};
    index i = first_index(k);
    long count = 0;
    while (true) {
      if (count++ == 200) {std::cout << size() << ", " << m << std::endl; abort();}
      ptr old = H[i].load();
      if (old == 0 &&
	  H[i].compare_exchange_strong(old, kv))
	return true;
      if (deref(old)->first == k) {
	if (is_deleted(old) &&
	    H[i].compare_exchange_strong(old, kv)) {
	  deref(old)->first.~K();
	  parlay::p_free(deref(old));
	  return true;
	} else {
	  deref(kv)->~KV();
	  parlay::p_free(deref(kv));
	  return false;
	}
      }
      i = next_index(i);
    }
  }

  V& at(const K& k) {
    index i = first_index(k);
    long count = 0;
    while (true) {
      if (count++ == 200) {std::cout << size() << ", " << m << std::endl; abort();}
      ptr p = H[i].load();
      if (p == 0) abort();
      if (deref(p)->first == k) return deref(p)->second;
      i = next_index(i);
    }
  }

  std::optional<V> remove(const K& k) {
    index i = first_index(k);
    long count = 0;
    while (true) {
      if (count++ == 200) {std::cout << size() << ", " << m << std::endl; abort();}
      ptr p = H[i].load();
      if (p == 0) return {};
      if (deref(p)->first == k) {
	if (is_deleted(p)) return {};
	V result = std::move(deref(p)->second);
	return result;
      }
      i = next_index(i);
    }
  }

  parlay::sequence<KV*> entries() {
      return parlay::filter(parlay::delayed_map(H, [] (auto const &x) {return x.load();}),
			    [] (KV* ptr) {return ptr != nullptr;});
  }

  size_t size() {
      return parlay::reduce(parlay::delayed_map(H, [&] (auto const &x) -> long {
	    ptr p = x.load();
	    return x != 0 && !is_deleted(x);}));
  }
};
  
    
