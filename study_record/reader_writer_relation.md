## 我们由channel_test_writer简单程序分析
```cpp
cyber/examples/common_component_example/channel_test_writer.cc:26~47
int main(int argc, char *argv[]) {
  // init cyber framework
  apollo::cyber::Init(argv[0]);
  // create talker node
  auto talker_node = apollo::cyber::CreateNode("channel_test_writer");  //#1 创建Node
  // create talker
  auto talker = talker_node->CreateWriter<Driver>("/apollo/test");      //#2 创建Writer
  Rate rate(2.0);

  std::string content("apollo_test");
  while (apollo::cyber::OK()) {
    static uint64_t seq = 0;
    auto msg = std::make_shared<Driver>();
    msg->set_timestamp(Time::Now().ToNanosecond());
    msg->set_msg_id(seq++);
    msg->set_content(content + std::to_string(seq - 1));
    talker->Write(msg);                                              //#3 调用Write接口写消息
    AINFO << "/apollo/test sent message, seq=" << (seq - 1) << ";";
    rate.Sleep();
  }
  return 0;
}

```
我们看到channel_test_writer核心的操作是#1 创建Node,#2 创建Writer,#3 调用Write接口写消息，下面我们一一分析

## CreateNode源码实现
```cpp
std::unique_ptr<Node> CreateNode(const std::string& node_name,
                                 const std::string& name_space) {
  bool is_reality_mode = GlobalData::Instance()->IsRealityMode();
  if (is_reality_mode && !OK()) {
    // add some hint log
    AERROR << "please initialize cyber firstly.";
    return nullptr;
  }
  return std::unique_ptr<Node>(new Node(node_name, name_space));
}
```
上面代码实际上创建了Node.

## Node相关的类
```cpp
cyber/node/node.h:44~186
class Node {
 public:
  template <typename M0, typename M1, typename M2, typename M3>
  friend class Component;
  friend class TimerComponent;
  friend bool Init(const char*);
  friend std::unique_ptr<Node> CreateNode(const std::string&,
                                          const std::string&);
  virtual ~Node();

...
  template <typename MessageT>
  auto CreateWriter(const proto::RoleAttributes& role_attr)
      -> std::shared_ptr<Writer<MessageT>>;

...
  template <typename MessageT>
  auto CreateWriter(const std::string& channel_name)
      -> std::shared_ptr<Writer<MessageT>>;

...
  template <typename MessageT>
  auto CreateReader(const std::string& channel_name,
                    const CallbackFunc<MessageT>& reader_func = nullptr)
      -> std::shared_ptr<cyber::Reader<MessageT>>;

...
  template <typename MessageT>
  auto CreateReader(const ReaderConfig& config,
                    const CallbackFunc<MessageT>& reader_func = nullptr)
      -> std::shared_ptr<cyber::Reader<MessageT>>;

...
  template <typename MessageT>
  auto CreateReader(const proto::RoleAttributes& role_attr,
                    const CallbackFunc<MessageT>& reader_func = nullptr)
      -> std::shared_ptr<cyber::Reader<MessageT>>;

...
  template <typename Request, typename Response>
  auto CreateService(const std::string& service_name,
                     const typename Service<Request, Response>::ServiceCallback&
                         service_callback)
      -> std::shared_ptr<Service<Request, Response>>;

...
  template <typename Request, typename Response>
  auto CreateClient(const std::string& service_name)
      -> std::shared_ptr<Client<Request, Response>>;

  /**
   * @brief Observe all readers' data
   */
  void Observe();

...

 private:
  explicit Node(const std::string& node_name,
                const std::string& name_space = "");

  std::string node_name_;
  std::string name_space_;

  std::mutex readers_mutex_;
  std::map<std::string, std::shared_ptr<ReaderBase>> readers_;

  std::unique_ptr<NodeChannelImpl> node_channel_impl_ = nullptr;
  std::unique_ptr<NodeServiceImpl> node_service_impl_ = nullptr;
};

Node::Node(const std::string& node_name, const std::string& name_space)
    : node_name_(node_name), name_space_(name_space) {
  node_channel_impl_.reset(new NodeChannelImpl(node_name));
  node_service_impl_.reset(new NodeServiceImpl(node_name));
}

//#1 CreateWriter源码实现
template <typename MessageT>
auto Node::CreateWriter(const std::string& channel_name)
    -> std::shared_ptr<Writer<MessageT>> {
  return node_channel_impl_->template CreateWriter<MessageT>(channel_name);
}

//#2 CreateReader源码实现
template <typename MessageT>
auto Node::CreateReader(const std::string& channel_name,
                        const CallbackFunc<MessageT>& reader_func)
    -> std::shared_ptr<Reader<MessageT>> {
  std::lock_guard<std::mutex> lg(readers_mutex_);
  if (readers_.find(channel_name) != readers_.end()) {
    AWARN << "Failed to create reader: reader with the same channel already "
             "exists.";
    return nullptr;
  }
  auto reader = node_channel_impl_->template CreateReader<MessageT>(
      channel_name, reader_func);
  if (reader != nullptr) {
    readers_.emplace(std::make_pair(channel_name, reader));
  }
  return reader;
}
```
通过类成员变量以及实现方法可以发现，Node类组合了NodeChannelImpl以及NodeServiceImpl，并将实现委托给了这两个类。
### NodeChannelImpl实现
```
template <typename MessageT>
auto NodeChannelImpl::CreateWriter(const proto::RoleAttributes& role_attr)
    -> std::shared_ptr<Writer<MessageT>> {
  if (!role_attr.has_channel_name() || role_attr.channel_name().empty()) {
    AERROR << "Can't create a writer with empty channel name!";
    return nullptr;
  }
  proto::RoleAttributes new_attr(role_attr);
  FillInAttr<MessageT>(&new_attr);

  std::shared_ptr<Writer<MessageT>> writer_ptr = nullptr;
  if (!is_reality_mode_) {
    writer_ptr = std::make_shared<blocker::IntraWriter<MessageT>>(new_attr);
  } else {
    writer_ptr = std::make_shared<Writer<MessageT>>(new_attr);    //#1 创建Writer<MessageT>对象
  }

  RETURN_VAL_IF_NULL(writer_ptr, nullptr);
  RETURN_VAL_IF(!writer_ptr->Init(), nullptr);                    //#2 调用初始化函数
  return writer_ptr;
}

template <typename MessageT>
auto NodeChannelImpl::CreateReader(const proto::RoleAttributes& role_attr,
                                   const CallbackFunc<MessageT>& reader_func,
                                   uint32_t pending_queue_size)
    -> std::shared_ptr<Reader<MessageT>> {
  if (!role_attr.has_channel_name() || role_attr.channel_name().empty()) {
    AERROR << "Can't create a reader with empty channel name!";
    return nullptr;
  }

  proto::RoleAttributes new_attr(role_attr);
  FillInAttr<MessageT>(&new_attr);

  std::shared_ptr<Reader<MessageT>> reader_ptr = nullptr;
  if (!is_reality_mode_) {
    reader_ptr =
        std::make_shared<blocker::IntraReader<MessageT>>(new_attr, reader_func);
  } else {
    reader_ptr = std::make_shared<Reader<MessageT>>(new_attr, reader_func, //#1 创建一个新的Reader<MessageT>对象
                                                    pending_queue_size);
  }

  RETURN_VAL_IF_NULL(reader_ptr, nullptr);
  RETURN_VAL_IF(!reader_ptr->Init(), nullptr);                            //#2 调用初始化函数
  return reader_ptr;
}
```

### Writer类实现
```cpp
template <typename MessageT>
Writer<MessageT>::Writer(const proto::RoleAttributes& role_attr)
    : WriterBase(role_attr), transmitter_(nullptr), channel_manager_(nullptr) {}

template <typename MessageT>
Writer<MessageT>::~Writer() {
  Shutdown();
}

template <typename MessageT>
bool Writer<MessageT>::Init() {
  {
    std::lock_guard<std::mutex> g(lock_);
    if (init_) {
      return true;
    }
    transmitter_ =
        transport::Transport::Instance()->CreateTransmitter<MessageT>( //#1 创建CreateTransmitter
            role_attr_);
    if (transmitter_ == nullptr) {
      return false;
    }
    init_ = true;
  }
  this->role_attr_.set_id(transmitter_->id().HashValue());
  channel_manager_ =
      service_discovery::TopologyManager::Instance()->channel_manager(); //#2 获取channel_manager
  JoinTheTopology();                                                     //#3 加入拓扑中
  return true;
}

template <typename MessageT>
void Writer<MessageT>::JoinTheTopology() {
  // add listener
  change_conn_ = channel_manager_->AddChangeListener(std::bind(
      &Writer<MessageT>::OnChannelChange, this, std::placeholders::_1)); //#4设置ChannelChange回调

  // get peer readers
  const std::string& channel_name = this->role_attr_.channel_name();
  std::vector<proto::RoleAttributes> readers;
  channel_manager_->GetReadersOfChannel(channel_name, &readers);         //#5 获取所有的Readers并Enable  
  for (auto& reader : readers) {
    transmitter_->Enable(reader);
  }

  channel_manager_->Join(this->role_attr_, proto::RoleType::ROLE_WRITER, //#6 将本Writer设置为写角色
                         message::HasSerializer<MessageT>::value);
}

template <typename MessageT>
void Writer<MessageT>::OnChannelChange(const proto::ChangeMsg& change_msg) {
  if (change_msg.role_type() != proto::RoleType::ROLE_READER) {
    return;
  }

  auto& reader_attr = change_msg.role_attr();
  if (reader_attr.channel_name() != this->role_attr_.channel_name()) {
    return;
  }

  auto operate_type = change_msg.operate_type();
  if (operate_type == proto::OperateType::OPT_JOIN) {
    transmitter_->Enable(reader_attr);
  } else {
    transmitter_->Disable(reader_attr);
  }
}
```
上面代码可以看出Writer所做动作：创建CreateTransmitter，获取channel_manager，加入拓扑。其中加入拓扑动作完成了：设置ChannelChange回调，获取所有的Readers并Enable ，将本Writer设置为写角色。

### Reader类实现
```cpp
template <typename MessageT>
Reader<MessageT>::Reader(const proto::RoleAttributes& role_attr,
                         const CallbackFunc<MessageT>& reader_func,
                         uint32_t pending_queue_size)
    : ReaderBase(role_attr),
      pending_queue_size_(pending_queue_size),
      reader_func_(reader_func) {
  blocker_.reset(new blocker::Blocker<MessageT>(blocker::BlockerAttr(
      role_attr.qos_profile().depth(), role_attr.channel_name())));
}

template <typename MessageT>
bool Reader<MessageT>::Init() {
  if (init_.exchange(true)) {
    return true;
  }
  std::function<void(const std::shared_ptr<MessageT>&)> func;
  if (reader_func_ != nullptr) {
    func = [this](const std::shared_ptr<MessageT>& msg) {             //#1 创建func lambda表达式
      this->Enqueue(msg);
      this->reader_func_(msg);
    };
  } else {
    func = [this](const std::shared_ptr<MessageT>& msg) { this->Enqueue(msg); };
  }
  auto sched = scheduler::Instance();
  croutine_name_ = role_attr_.node_name() + "_" + role_attr_.channel_name();
  auto dv = std::make_shared<data::DataVisitor<MessageT>>(          //#2 创建DataVisitor
      role_attr_.channel_id(), pending_queue_size_);
  // Using factory to wrap templates.
  croutine::RoutineFactory factory =
      croutine::CreateRoutineFactory<MessageT>(std::move(func), dv);   //#2 创建协程Task
  if (!sched->CreateTask(factory, croutine_name_)) {
    AERROR << "Create Task Failed!";
    init_.store(false);
    return false;
  }

  receiver_ = ReceiverManager<MessageT>::Instance()->GetReceiver(role_attr_);
  this->role_attr_.set_id(receiver_->id().HashValue());
  channel_manager_ =
      service_discovery::TopologyManager::Instance()->channel_manager();
  JoinTheTopology();                                              //#3 加入拓扑

  return true;
}

template <typename MessageT>
auto ReceiverManager<MessageT>::GetReceiver(
    const proto::RoleAttributes& role_attr) ->
    typename std::shared_ptr<transport::Receiver<MessageT>> {
  std::lock_guard<std::mutex> lock(receiver_map_mutex_);
  // because multi reader for one channel will write datacache multi times,
  // so reader for datacache we use map to keep one instance for per channel
  const std::string& channel_name = role_attr.channel_name();
  if (receiver_map_.count(channel_name) == 0) {
    receiver_map_[channel_name] =
        transport::Transport::Instance()->CreateReceiver<MessageT>( //# 1创建CreateReceiver
            role_attr, [](const std::shared_ptr<MessageT>& msg,
                          const transport::MessageInfo& msg_info,
                          const proto::RoleAttributes& reader_attr) {
              (void)msg_info;
              (void)reader_attr;
              PerfEventCache::Instance()->AddTransportEvent(
                  TransPerf::DISPATCH, reader_attr.channel_id(),
                  msg_info.seq_num());
              data::DataDispatcher<MessageT>::Instance()->Dispatch( //#2 有数据来的时候调用分发
                  reader_attr.channel_id(), msg);
              PerfEventCache::Instance()->AddTransportEvent(
                  TransPerf::NOTIFY, reader_attr.channel_id(),
                  msg_info.seq_num());
            });
  }
  return receiver_map_[channel_name];
}

template <typename MessageT>
void Reader<MessageT>::JoinTheTopology() {
  // add listener
  change_conn_ = channel_manager_->AddChangeListener(std::bind(
      &Reader<MessageT>::OnChannelChange, this, std::placeholders::_1));

  // get peer writers
  const std::string& channel_name = this->role_attr_.channel_name();
  std::vector<proto::RoleAttributes> writers;
  channel_manager_->GetWritersOfChannel(channel_name, &writers);
  for (auto& writer : writers) {
    receiver_->Enable(writer);
  }
  channel_manager_->Join(this->role_attr_, proto::RoleType::ROLE_READER,
                         message::HasSerializer<MessageT>::value);
}

template <typename MessageT>
void Reader<MessageT>::OnChannelChange(const proto::ChangeMsg& change_msg) {
  if (change_msg.role_type() != proto::RoleType::ROLE_WRITER) {
    return;
  }

  auto& writer_attr = change_msg.role_attr();
  if (writer_attr.channel_name() != this->role_attr_.channel_name()) {
    return;
  }

  auto operate_type = change_msg.operate_type();
  if (operate_type == proto::OperateType::OPT_JOIN) {
    receiver_->Enable(writer_attr);
  } else {
    receiver_->Disable(writer_attr);
  }
}
```
上面代码可以看出Reader所做动作：创建CreateReceiver，获取channel_manager，加入拓扑。其中加入拓扑动作完成了：设置ChannelChange回调，获取所有的Writers并Enable ，将本Reader设置为写角色。

通过上面的分析Node类通过CreateWriter,CreateReader，并通过channel_name完成消息的相互关联