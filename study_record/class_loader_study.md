class_loader分析

## 背景引入
在mainboard分析中我们留了两处需要进一步分析的地方：
    1.分析加载dag/module_config/module_library:class_loader::ClassLoaderManager class_loader_manager_; class_loader_manager_.LoadLibrary(load_path);
    2.分析加载dag/module_config/components/class_name的过程:std::shared_ptr<ComponentBase> base = class_loader_manager_.CreateClassObj<ComponentBase> (class_name);
    要解开上面两个疑问，需要进一步分析class_loader模块，其源吗在cyber/class_loader。


## 第一个疑问：class_loader_manager_.LoadLibrary(load_path)源码分析
```
bool ClassLoaderManager::LoadLibrary(const std::string& library_path) {
  std::lock_guard<std::mutex> lck(libpath_loader_map_mutex_);
  //第一次调用时无效，创建ClassLoader
  if (!IsLibraryValid(library_path)) {
    libpath_loader_map_[library_path] =
        new class_loader::ClassLoader(library_path);
  }
  return IsLibraryValid(library_path);
}

ClassLoader::ClassLoader(const std::string& library_path)
    : library_path_(library_path),
      loadlib_ref_count_(0),
      classobj_ref_count_(0) {
  //创建ClassLoader会直接加载Library
  LoadLibrary();
}

bool ClassLoader::LoadLibrary() {
  std::lock_guard<std::mutex> lck(loadlib_ref_count_mutex_);
  ++loadlib_ref_count_;
  AINFO << "Begin LoadLibrary: " << library_path_;
  //进一步调用辅助类的LoadLibrary
  return utility::LoadLibrary(library_path_, this);
}

bool LoadLibrary(const std::string& library_path, ClassLoader* loader) {
  //判断是否已经被打开
  if (IsLibraryLoadedByAnybody(library_path)) {
    AINFO << "lib has been loaded by others,only attach to class factory obj."
          << library_path;
    ClassFactoryVector lib_class_factory_objs =
        GetAllClassFactoryObjectsOfLibrary(library_path);
    for (auto& class_factory_obj : lib_class_factory_objs) {
      class_factory_obj->AddOwnedClassLoader(loader);
    }
    return true;
  }

  SharedLibraryPtr shared_library = nullptr;
  static std::recursive_mutex loader_mutex;
  {
    std::lock_guard<std::recursive_mutex> lck(loader_mutex);

    try {
      //设置当前的class_loader library_path，创建SharedLibrary
      SetCurActiveClassLoader(loader);
      SetCurLoadingLibraryName(library_path);
      shared_library = SharedLibraryPtr(new SharedLibrary(library_path));
    } catch (const LibraryLoadException& e) {
     ...
    }

    SetCurLoadingLibraryName("");
    SetCurActiveClassLoader(nullptr);
  }

...

  //放入到opened_libraries中记录
  std::lock_guard<std::recursive_mutex> lck(GetLibPathSharedLibMutex());
  LibPathSharedLibVector& opened_libraries = GetLibPathSharedLibVector();
  opened_libraries.emplace_back(
      std::pair<std::string, SharedLibraryPtr>(library_path, shared_library));
  return true;
}

```
SharedLibrary类又完成了什么呢？我们继续分析。
### shared_library完成了动态库加载的核心动作
源码在cyber/class_loader/shared_library

```cpp

class SharedLibrary {
    ...
  // Loads a shared library from the given path,
  // using the given flags. See the Flags enumeration
  // for valid values.
  // Throws a LibraryAlreadyLoadedException if
  // a library has already been loaded.
  // Throws a LibraryLoadException if the library
  // cannot be loaded.
  void Load(const std::string& path, int flags);

  // Unloads a shared library.
  void Unload();
...
  // Returns the address of the symbol with
  // the given name. For functions, this
  // is the entry point of the function.
  // Throws a SymbolNotFoundException if the
  // symbol does not exist.
  void* GetSymbol(const std::string& name);
...

};

SharedLibrary::SharedLibrary(const std::string& path) { Load(path, 0); }


void SharedLibrary::Load(const std::string& path, int flags) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (handle_) throw LibraryAlreadyLoadedException(path);

  int real_flag = RTLD_LAZY;
  if (flags & SHLIB_LOCAL) {
    real_flag |= RTLD_LOCAL;
  } else {
    real_flag |= RTLD_GLOBAL;
  }
  //调用系统接口dlopen打开library
  handle_ = dlopen(path.c_str(), real_flag);
  if (!handle_) {
    const char* err = dlerror();
    throw LibraryLoadException(err ? std::string(err) : path);
  }

  path_ = path;
}

void SharedLibrary::Unload() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (handle_) {
    //调用系统接口dlclose关闭library
    dlclose(handle_);
    handle_ = nullptr;
  }
}

void* SharedLibrary::GetSymbol(const std::string& name) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!handle_) return nullptr;

  //调用dlsym查看是否有响应的符号
  void* result = dlsym(handle_, name.c_str());
  if (!result) {
    throw SymbolNotFoundException(name);
  }

  return result;
}
```
我们看到SharedLibrary类封装了Load Unload GetSymbol接口，调用dlopen  dlclose dlsym，这三个接口可以参考https://www.cnblogs.com/youxin/p/5109520.html
    dlopen:功能：打开一个动态链接库 
    dlsym根据动态链接库操作句柄(handle)与符号(symbol)，返回符号对应的地址。使用这个函数不但可以获取函数地址，也可以获取变量地址。
    dlclose()dlclose用于关闭指定句柄的动态链接库，只有当此动态链接库的使用计数为0时,才会真正被系统卸载。
    dlerror()当动态链接库操作函数执行失败时，dlerror可以返回出错信息，返回值为NULL时表示操作函数执行成功

## 第二个问题源码分析
我们先看一下CYBER_REGISTER_COMPONENT注册过程
### CYBER_REGISTER_COMPONENT(CommonComponentSample)注册

```cpp

#define CYBER_REGISTER_COMPONENT(name) \
  CLASS_LOADER_REGISTER_CLASS(name, apollo::cyber::ComponentBase)


#define CLASS_LOADER_REGISTER_CLASS_INTERNAL(Derived, Base, UniqueID)     \
  namespace {                                                             \
  struct ProxyType##UniqueID {                                            \
    ProxyType##UniqueID() {                                               \
      apollo::cyber::class_loader::utility::RegisterClass<Derived, Base>( \
          #Derived, #Base);                                               \
    }                                                                     \
  };                                                                      \
  static ProxyType##UniqueID g_register_class_##UniqueID;                 \
  }

#define CLASS_LOADER_REGISTER_CLASS_INTERNAL_1(Derived, Base, UniqueID) \
  CLASS_LOADER_REGISTER_CLASS_INTERNAL(Derived, Base, UniqueID)

// register class macro
#define CLASS_LOADER_REGISTER_CLASS(Derived, Base) \
  CLASS_LOADER_REGISTER_CLASS_INTERNAL_1(Derived, Base, __COUNTER__)

template <typename Derived, typename Base>
void RegisterClass(const std::string& class_name,
                   const std::string& base_class_name) {
  AINFO << "registerclass:" << class_name << "," << base_class_name << ","
        << GetCurLoadingLibraryName();

  utility::AbstractClassFactory<Base>* new_class_factory_obj =
      new utility::ClassFactory<Derived, Base>(class_name, base_class_name);
  new_class_factory_obj->AddOwnedClassLoader(GetCurActiveClassLoader());
  new_class_factory_obj->SetRelativeLibraryPath(GetCurLoadingLibraryName());

  GetClassFactoryMapMapMutex().lock();
  ClassClassFactoryMap& factory_map =
      GetClassFactoryMapByBaseClass(typeid(Base).name());
  factory_map[class_name] = new_class_factory_obj;
  GetClassFactoryMapMapMutex().unlock();
}
```
通过上述调用，apollo::cyber::class_loader::utility::RegisterClass<Derived, Base>( #Derived, #Base)，最终调用 new utility::ClassFactory<Derived, Base>(class_name, base_class_name)并放入factory_map中。

### 第二个问题创建新的CreateClassObj源码分析
std::shared_ptr<ComponentBase> base = class_loader_manager_.CreateClassObj<ComponentBase> (class_name);
```cpp
template <typename Base>
std::shared_ptr<Base> ClassLoaderManager::CreateClassObj(
    const std::string& class_name) {
  std::vector<ClassLoader*> class_loaders = GetAllValidClassLoaders();
  for (auto class_loader : class_loaders) {
    if (class_loader->IsClassValid<Base>(class_name)) {
      return (class_loader->CreateClassObj<Base>(class_name));
    }
  }
  AERROR << "Invalid class name: " << class_name;
  return std::shared_ptr<Base>();
}

template <typename Base>
std::shared_ptr<Base> ClassLoader::CreateClassObj(
    const std::string& class_name) {
  if (!IsLibraryLoaded()) {
    LoadLibrary();
  }

  Base* class_object = utility::CreateClassObj<Base>(class_name, this);
  if (class_object == nullptr) {
    AWARN << "CreateClassObj failed, ensure class has been registered. "
          << "classname: " << class_name << ",lib: " << GetLibraryPath();
    return std::shared_ptr<Base>();
  }

  std::lock_guard<std::mutex> lck(classobj_ref_count_mutex_);
  classobj_ref_count_ = classobj_ref_count_ + 1;
  std::shared_ptr<Base> classObjSharePtr(
      class_object, std::bind(&ClassLoader::OnClassObjDeleter<Base>, this,
                              std::placeholders::_1));
  return classObjSharePtr;
}

template <typename Base>
Base* CreateClassObj(const std::string& class_name, ClassLoader* loader) {
  GetClassFactoryMapMapMutex().lock();
  ClassClassFactoryMap& factoryMap =
      GetClassFactoryMapByBaseClass(typeid(Base).name());
  AbstractClassFactory<Base>* factory = nullptr;
  if (factoryMap.find(class_name) != factoryMap.end()) {
    factory = dynamic_cast<utility::AbstractClassFactory<Base>*>(
        factoryMap[class_name]);
  }
  GetClassFactoryMapMapMutex().unlock();

  Base* classobj = nullptr;
  if (factory && factory->IsOwnedBy(loader)) {
    classobj = factory->CreateObj();
  }

  return classobj;
}

template <typename ClassObject, typename Base>
class ClassFactory : public AbstractClassFactory<Base> {
 public:
  ClassFactory(const std::string& class_name,
               const std::string& base_class_name)
      : AbstractClassFactory<Base>(class_name, base_class_name) {}

  Base* CreateObj() const { return new ClassObject; }
};
```
CreateClassObj一路调用，最终从factoryMap获取到facoory并调用CreateObj。


