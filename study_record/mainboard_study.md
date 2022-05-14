
## 我们从cyber/doxy-docs/source/CyberRT_Quick_Start.md文档中可以知道如何加载一个component:

```shell
cyber_launch start cyber/examples/common_component_example/common.launch
or
mainboard -d cyber/examples/common_component_example/common.dag
```

## 第一种cyber_launch日志如下：
```log
[cyber_launch_146531] INFO Launch file [/apollo/.cache/bazel/540135163923dd7d5820f3ee4b306b32/execroot/apollo/bazel-out/k8-fastbuild/bin/cyber/examples/common_component_example/install.runfiles/apollo/cyber/examples/common_component_example/./common.launch]
[cyber_launch_146531] INFO ========================================================================================================================
[cyber_launch_146531] INFO Load module [common] library: [common] [CYBER_DEFAULT] conf: [/apollo/cyber/examples/common_component_example/common.dag] exception_handler: []
[cyber_launch_146531] INFO Start process [common] successfully. pid: 146538
[cyber_launch_146531] INFO ------------------------------------------------------------------------------------------------------------------------
[common ]  WARNING: Logging before InitGoogleLogging() is written to STDERR
[common ]  I0514 10:44:35.723547 146538 module_argument.cc:81] []command: mainboard -d /apollo/cyber/examples/common_component_example/common.dag -p common -s CYBER_DEFAULT
[common ]  I0514 10:44:35.723976 146538 global_data.cc:153] []host ip: 172.20.1.88
[common ]  I0514 10:44:35.726163 146538 module_argument.cc:57] []binary_name_ is mainboard, process_group_ is common, has 1 dag conf
[common ]  I0514 10:44:35.726181 146538 module_argument.cc:60] []dag_conf: /apollo/cyber/examples/common_component_example/common.dag
```
可以看出来最后cyber_launch也是调用了 mainboard -d /apollo/cyber/examples/common_component_example/common.dag -p common -s CYBER_DEFAULT

# 那么我们来分析mainboard

```cpp
int main(int argc, char** argv) {
  // parse the argument
  ModuleArgument module_args;
  //解析参数
  module_args.ParseArgument(argc, argv);

  // initialize cyber
  //初始化cyber
  apollo::cyber::Init(argv[0]);

  // start module
  //启动module
  ModuleController controller(module_args);
  if (!controller.Init()) {
    controller.Clear();
    AERROR << "module start error.";
    return -1;
  }

  //等待shutdown
  apollo::cyber::WaitForShutdown();
  controller.Clear();
  AINFO << "exit mainboard.";

  return 0;
}
```
可以看到mainboard main函数中做参数解析、初始化cyber、启动module、等待module结束,这就是mainboard加载运行so库的过程了。

## 参数解析源码module_args.ParseArgument(argc, argv)分析：
 ```cpp
 void ModuleArgument::ParseArgument(const int argc, char* const argv[]) {
  binary_name_ = std::string(basename(argv[0]));
  //解析-d参数到dag_conf_list_ -p参数到process_group_ -s参数到sched_name_
  GetOptions(argc, argv);

  if (process_group_.empty()) {
    process_group_ = DEFAULT_process_group_;
  }

  if (sched_name_.empty()) {
    sched_name_ = DEFAULT_sched_name_;
  }
  
  //设置process_group_ sched_name_
  GlobalData::Instance()->SetProcessGroup(process_group_);
  GlobalData::Instance()->SetSchedName(sched_name_);
  AINFO << "binary_name_ is " << binary_name_ << ", process_group_ is "
        << process_group_ << ", has " << dag_conf_list_.size() << " dag conf";
  for (std::string& dag : dag_conf_list_) {
    AINFO << "dag_conf: " << dag;
  }
}
 ```
解析-d参数到dag_conf_list_ -p参数到process_group_ -s参数到sched_name_，并设置设置process_group_ sched_name_


## cyber初始化apollo::cyber::Init(argv[0])源码分析

```cpp
bool Init(const char* binary_name) {
  std::lock_guard<std::mutex> lg(g_mutex);
  if (GetState() != STATE_UNINITIALIZED) {
    return false;
  }

  //TODO: 需要分析初始化log
  InitLogger(binary_name);
  auto thread = const_cast<std::thread*>(async_logger->LogThread());
  scheduler::Instance()->SetInnerThreadAttr("async_log", thread);

  //TODO:需要分析 初始化系统监控
  SysMo::Instance();

  //设置ctrl+c signal
  std::signal(SIGINT, OnShutdown);
  // Register exit handlers
  if (!g_atexit_registered) {
    //设置exit时做的工作,程序退出时会执行ExitHandle
    if (std::atexit(ExitHandle) != 0) {
      AERROR << "Register exit handle failed";
      return false;
    }
    AINFO << "Register exit handle succ.";
    g_atexit_registered = true;
  }
  SetState(STATE_INITIALIZED);

  //TODO:mock模式下的时钟，mock模式做什么用
  auto global_data = GlobalData::Instance();
  if (global_data->IsMockTimeMode()) {
    auto node_name = kClockNode + std::to_string(getpid());
    clock_node = std::unique_ptr<Node>(new Node(node_name));
    auto cb =
        [](const std::shared_ptr<const apollo::cyber::proto::Clock>& msg) {
          if (msg->has_clock()) {
            Clock::Instance()->SetNow(Time(msg->clock()));
          }
        };
    clock_node->CreateReader<apollo::cyber::proto::Clock>(kClockChannel, cb);
  }
  return true;
}
```
Init做初始化log,设置SIGINT相应ctrl+c，设置程序退出时会执行ExitHandle，mock模式下的时钟

## 模块控制器ModuleController controller(module_args) controller.Init()源码分析

controller.Init会调用ModuleController::LoadAll，其源码如下

```cpp
bool ModuleController::LoadAll() {
  const std::string work_root = common::WorkRoot();
  const std::string current_path = common::GetCurrentPath();
  const std::string dag_root_path = common::GetAbsolutePath(work_root, "dag");
  std::vector<std::string> paths;
  //每个dag文件，放入paths，并统计component_nums
  for (auto& dag_conf : args_.GetDAGConfList()) {
    std::string module_path = "";
    if (dag_conf == common::GetFileName(dag_conf)) {
      // case dag conf argument var is a filename
      module_path = common::GetAbsolutePath(dag_root_path, dag_conf);
    } else if (dag_conf[0] == '/') {
      // case dag conf argument var is an absolute path
      module_path = dag_conf;
    } else {
      // case dag conf argument var is a relative path
      module_path = common::GetAbsolutePath(current_path, dag_conf);
      if (!common::PathExists(module_path)) {
        module_path = common::GetAbsolutePath(work_root, dag_conf);
      }
    }
    total_component_nums += GetComponentNum(module_path);
    paths.emplace_back(std::move(module_path));
  }
  //TODO: 为什么has_timer_component=true时加上TaskPoolSize
  if (has_timer_component) {
    total_component_nums += scheduler::Instance()->TaskPoolSize();
  }
  common::GlobalData::Instance()->SetComponentNums(total_component_nums);
  for (auto module_path : paths) {
    AINFO << "Start initialize dag: " << module_path;
    //加载每个module
    if (!LoadModule(module_path)) {
      AERROR << "Failed to load module: " << module_path;
      return false;
    }
  }
  return true;
}
```
每个dag文件，放入paths，并统计component_nums;加载每个module。

### ModuleController::LoadModule如何做的呢？
```cpp
bool ModuleController::LoadModule(const DagConfig& dag_config) {
  const std::string work_root = common::WorkRoot();

  for (auto module_config : dag_config.module_config()) {
    //解析dag/module_config/module_library
    std::string load_path;
    if (module_config.module_library().front() == '/') {
      load_path = module_config.module_library();
    } else {
      load_path =
          common::GetAbsolutePath(work_root, module_config.module_library());
    }

    if (!common::PathExists(load_path)) {
      AERROR << "Path does not exist: " << load_path;
      return false;
    }

    //TODO: 需要分析加载dag/module_config/module_library
    class_loader_manager_.LoadLibrary(load_path);

    for (auto& component : module_config.components()) {
      const std::string& class_name = component.class_name();
      //TODO: 需要分析加载dag/module_config/components/class_name的过程
      std::shared_ptr<ComponentBase> base =
          class_loader_manager_.CreateClassObj<ComponentBase>(class_name);
      //调用ComponentBase Initialize
      if (base == nullptr || !base->Initialize(component.config())) {
        return false;
      }
      component_list_.emplace_back(std::move(base));
    }

    for (auto& component : module_config.timer_components()) {
      const std::string& class_name = component.class_name();
      std::shared_ptr<ComponentBase> base =
          class_loader_manager_.CreateClassObj<ComponentBase>(class_name);
      if (base == nullptr || !base->Initialize(component.config())) {
        return false;
      }
      component_list_.emplace_back(std::move(base));
    }
  }
  return true;
}
```

解析加载dag/module_config/module_library
加载dag/module_config/components/class_name
调用调用ComponentBase Initialize
### ComponentBase Initialize分析
```cpp
class ComponentBase : public std::enable_shared_from_this<ComponentBase> {
 public:
...
  virtual bool Initialize(const ComponentConfig& config) { return false; }
...
 protected:
  virtual bool Init() = 0;
...
}

template <typename M0, typename M1>
class Component<M0, M1, NullType, NullType> : public ComponentBase {
 public:
  Component() {}
  ~Component() override {}
  bool Initialize(const ComponentConfig& config) override;
  bool Process(const std::shared_ptr<M0>& msg0,
               const std::shared_ptr<M1>& msg1);

 private:
  virtual bool Proc(const std::shared_ptr<M0>& msg,
                    const std::shared_ptr<M1>& msg1) = 0;
};

bool Component<M0, M1, NullType, NullType>::Initialize(
    const ComponentConfig& config) {
  node_.reset(new Node(config.name()));
  LoadConfigFiles(config);

  if (config.readers_size() < 2) {
    AERROR << "Invalid config file: too few readers.";
    return false;
  }

  if (!Init()) {
    AERROR << "Component Init() failed.";
    return false;
  }
  ...
}
template <typename M0, typename M1>
bool Component<M0, M1, NullType, NullType>::Process(
    const std::shared_ptr<M0>& msg0, const std::shared_ptr<M1>& msg1) {
  if (is_shutdown_.load()) {
    return true;
  }
  return Proc(msg0, msg1);
}

class CommonComponentSample : public Component<Driver, Driver> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<Driver>& msg0,
            const std::shared_ptr<Driver>& msg1) override;
};
```

```
bool CommonComponentSample::Init() {
  AINFO << "Commontest component init";
  return true;
}


bool CommonComponentSample::Proc(const std::shared_ptr<Driver>& msg0,
                                 const std::shared_ptr<Driver>& msg1) {
  AINFO << "Start common component Proc [" << msg0->msg_id() << "] ["
        << msg1->msg_id() << "]";
  return true;
}
```
从上述代码可以看出来CommonComponentSample继承Component，Component继承ComponentBase。
虚函数base->Initialize(component.config()最终调用CommonComponentSample::Init();输出Commontest component init
另外当有数据时会调用base->Proc()最终调用CommonComponentSample::Proc。输出Start common component Proc...

## common_component_example执行日志如下
```log
Log file created at: 2022/05/14 10:50:27
Running on machine: in-dev-docker
Log line format: [IWEF]mmdd hh:mm:ss.uuuuuu threadid file:line] msg
W0514 10:50:27.784358 148465 scheduler_factory.cc:63] Scheduler conf named /apollo/cyber/conf/common.conf not found, use default.
I0514 10:50:27.784595 148468 processor.cc:42] processor_tid: 148468
I0514 10:50:27.784678 148469 processor.cc:42] processor_tid: 148469
I0514 10:50:27.784754 148470 processor.cc:42] processor_tid: 148470
I0514 10:50:27.784809 148471 processor.cc:42] processor_tid: 148471
I0514 10:50:27.784886 148472 processor.cc:42] processor_tid: 148472
I0514 10:50:27.784919 148473 processor.cc:42] processor_tid: 148473
I0514 10:50:27.784953 148474 processor.cc:42] processor_tid: 148474
I0514 10:50:27.785079 148475 processor.cc:42] processor_tid: 148475
I0514 10:50:27.785087 148476 processor.cc:42] processor_tid: 148476
I0514 10:50:27.785128 148477 processor.cc:42] processor_tid: 148477
I0514 10:50:27.785185 148478 processor.cc:42] processor_tid: 148478
I0514 10:50:27.785243 148479 processor.cc:42] processor_tid: 148479
I0514 10:50:27.785286 148480 processor.cc:42] processor_tid: 148480
I0514 10:50:27.785310 148481 processor.cc:42] processor_tid: 148481
I0514 10:50:27.785346 148482 processor.cc:42] processor_tid: 148482
I0514 10:50:27.785387 148465 init.cc:113] Register exit handle succ.
I0514 10:50:27.785398 148483 processor.cc:42] processor_tid: 148483
I0514 10:50:27.785943 148465 module_controller.cc:65] Start initialize dag: /apollo/cyber/examples/common_component_example/common.dag
I0514 10:50:27.786028 148465 class_loader.cc:37] Begin LoadLibrary: /apollo/bazel-bin/cyber/examples/common_component_example/libcommon_component_example.so
I0514 10:50:27.796957 148465 class_loader_utility.h:79] registerclass:CommonComponentSample,apollo::cyber::ComponentBase,/apollo/bazel-bin/cyber/examples/common_component_example/libcommon_component_example.so
I0514 10:50:27.801833 148465 common_component_example.cc:19] Commontest component init
I0514 10:50:27.802690 148465 scheduler.cc:55] create croutine: common_/apollo/test
I0514 10:50:27.803385 148465 scheduler.cc:55] create croutine: common_/apollo/prediction
I0514 10:50:27.803635 148465 scheduler.cc:55] create croutine: common
I0514 10:50:28.824244 148472 common_component_example.cc:25] Start common component Proc [191350] [127506]
I0514 10:50:29.157620 148475 common_component_example.cc:25] Start common component Proc [191351] [127507]
I0514 10:50:29.490818 148476 common_component_example.cc:25] Start common component Proc [191352] [127507]
I0514 10:50:29.824179 148472 common_component_example.cc:25] Start common component Proc [191353] [127508]
I0514 10:50:30.157466 148475 common_component_example.cc:25] Start common component Proc [191354] [127509]
I0514 10:50:30.490875 148477 common_component_example.cc:25] Start common component Proc [191355] [127509]
I0514 10:50:30.824224 148472 common_component_example.cc:25] Start common component Proc [191356] [127510]
I0514 10:50:31.157537 148475 common_component_example.cc:25] Start common component Proc [191357] [127511]
I0514 10:50:31.490917 148477 common_component_example.cc:25] Start common component Proc [191358] [127511]
I0514 10:50:31.824151 148472 common_component_example.cc:25] Start common component Proc [191359] [127512]
I0514 10:50:32.157471 148475 common_component_example.cc:25] Start common component Proc [191360] [127513]
I0514 10:50:32.490713 148477 common_component_example.cc:25] Start common component Proc [191361] [127513]
I0514 10:50:32.824231 148472 common_component_example.cc:25] Start common component Proc [191362] [127514]
I0514 10:50:33.157492 148475 common_component_example.cc:25] Start common component Proc [191363] [127515]
I0514 10:50:33.490787 148477 common_component_example.cc:25] Start common component Proc [191364] [127515]
I0514 10:50:33.824095 148471 common_component_example.cc:25] Start common component Proc [191365] [127516]
I0514 10:50:34.157315 148475 common_component_example.cc:25] Start common component Proc [191366] [127517]
I0514 10:50:34.207207 148465 mainboard.cc:45] exit mainboard.
```
从日志中我们可以看到
    Commontest component init
    Start common component Proc


## 待分析内容    
    分析初始化log
    分析 初始化系统监控
    mock模式下的时钟，mock模式做什么用
    为什么has_timer_component=true时加上TaskPoolSize
    分析加载dag/module_config/module_library
    分析加载dag/module_config/components/class_name的过程
    TODO需要进一步分析Init Proc如何调用的
