// 零 fork 的 protobuf typesupport：不修改 rclcpp / rcl / rmw / rosidl。
//
// ============================================================================
// 这个文件在做什么
// ============================================================================
// 让任意 protobuf 消息类型可以直接作为 ROS 2 的类型参数使用：
//
//   auto pub = node->create_publisher<my::ProtoMsg>("topic", rclcpp::QoS(10));
//   auto sub = node->create_subscription<my::ProtoMsg>("topic", ...);
//
// 实现方式是**只用 rosidl 预留的特化点**，不改任何 ROS 源码：
//   1) 特化 rosidl_generator_traits::is_message<T>       —— 声明「我是消息」
//   2) 特化 rosidl_typesupport_cpp::get_message_type_support_handle<T>()
//      —— 返回我们自建的 typesupport handle
//
// 这两处正是 rosidl 生成代码自己用的入口，所以走的是官方同一条路。
//
// ============================================================================
// 线上格式（wire format）
// ============================================================================
// 编码成单个 CDR sequence<uint8>：uint32 长度前缀 + protobuf 序列化字节。
// 与等价的 .msg 类型 `uint8[] data` 在 wire 上完全同形。
//
// ============================================================================
// Jazzy 的四道强制关卡（Humble 时代不存在，这是移植的主要坑）
// ============================================================================
// Jazzy 的 rcl/rmw 在建 pub/sub 时会**无条件解引用** handle 上的四个函数
// 指针。任何一个为空都直接 segfault（跳转到 0x0）或返回失败：
//
//   rmw TypeSupport_impl.cpp:612  → 必须能派生出 introspection handle，
//                                   否则 register_type_object 返回 false
//   rmw publisher.cpp:260         → get_type_hash_func
//   rcl publisher.c:133/134       → get_type_hash_func / get_type_description_func
//   rcl publisher.c:135           → get_type_description_sources_func
//
// 本文件把这四个全部补齐。
//
// ============================================================================
// 能力边界（务必知晓，不要误用）
// ============================================================================
// 合成的 introspection 描述的是**线上格式**（单个 uint8[]），不是 protobuf
// 的内存布局。注册路径全程不读 offset_ 与数组访问函数指针，所以安全；
// 但由此带来明确的限制：
//
//   - ros2 topic echo   ✗ 会按 uint8[] 显示裸字节，需专门的 typesupport 插件
//   - ros2 bag record   △ 能录，回放是裸字节
//   - RViz 可视化      ✗ 无法解析，需专门的 RViz 插件
//
// 这是本方案已知且接受的代价。若需要完整的自省能力，见
// docs/protobuf_migration_plan.md 中"typesupport 插件"一节。
#ifndef AMR_DISPATCHER_PROTO__PROTOBUF_TYPESUPPORT_HPP_
#define AMR_DISPATCHER_PROTO__PROTOBUF_TYPESUPPORT_HPP_

#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <type_traits>

#include <google/protobuf/message.h>
// FileDescriptorProto 在这里（type hash 用），message.h 不会带上它。
#include <google/protobuf/descriptor.pb.h>

#include <fastcdr/Cdr.h>

#include <rcutils/sha256.h>

#include <rosidl_runtime_c/message_type_support_struct.h>
#include <rosidl_runtime_c/type_hash.h>
#include <rosidl_runtime_c/type_description/field__struct.h>
#include <rosidl_runtime_c/type_description/field_type__struct.h>
#include <rosidl_runtime_c/type_description/individual_type_description__struct.h>
#include <rosidl_runtime_c/type_description/type_description__struct.h>
#include <rosidl_runtime_c/type_description/type_source__struct.h>
#include <rosidl_runtime_c/string.h>
#include <rosidl_runtime_c/string_functions.h>

#include <rosidl_runtime_cpp/traits.hpp>

#include <rosidl_typesupport_cpp/message_type_support.hpp>
#include <rosidl_typesupport_fastrtps_cpp/identifier.hpp>
#include <rosidl_typesupport_fastrtps_cpp/message_type_support.h>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

namespace amr_dispatcher_proto
{

/// 判断 T 是否为 protobuf 消息（用于 static_assert 给出清晰报错）。
template<typename T>
struct IsProtobufMessage : std::is_base_of<google::protobuf::Message, T>
{
};

namespace detail
{

/// proto package "a.b.c" -> DDS 命名空间 "a::b::c"
inline std::string ProtoNamespace(const std::string & full_name)
{
  const auto pos = full_name.find_last_of('.');
  if (pos == std::string::npos) {
    return "protobuf";
  }
  std::string ns = full_name.substr(0, pos);
  for (std::size_t i = 0; i < ns.size(); ++i) {
    if (ns[i] == '.') {
      ns.replace(i, 1, "::");
      ++i;
    }
  }
  return ns;
}

/// proto 全名 "a.b.C" -> 简单名 "C"
inline std::string ProtoName(const std::string & full_name)
{
  const auto pos = full_name.find_last_of('.');
  return pos == std::string::npos ? full_name : full_name.substr(pos + 1);
}

// ---------------------------------------------------------------------------
// type hash
// ---------------------------------------------------------------------------
// rcl 与 rmw 都会无条件解引用 get_type_hash_func（见文件头关卡说明），
// 所以这个函数指针不能留空——留空即跳转到 0x0 直接 segfault。
//
// hash 内容取自 protobuf 描述符的**规范化序列化**，而不是源码文本：
// FileDescriptorProto 的字段序与空白无关，因此同一份 .proto 无论重排
// 还是重新格式化，算出的 hash 都稳定——这正是类型 hash 需要的性质。
template<typename ProtoT>
const rosidl_type_hash_t * GetProtobufTypeHashFunc(
  const rosidl_message_type_support_t * /*handle*/)
{
  static const rosidl_type_hash_t hash = []() {
      google::protobuf::FileDescriptorProto file;
      ProtoT::descriptor()->file()->CopyTo(&file);
      std::string canonical;
      file.SerializeToString(&canonical);

      rosidl_type_hash_t out{};
      out.version = 1;  // RIHS01
      rcutils_sha256_ctx_t ctx;
      rcutils_sha256_init(&ctx);
      rcutils_sha256_update(
        &ctx, reinterpret_cast<const uint8_t *>(canonical.data()), canonical.size());
      rcutils_sha256_final(&ctx, out.value);
      return out;
    }();
  return &hash;
}

// ---------------------------------------------------------------------------
// type description
// ---------------------------------------------------------------------------
// rcl_publisher_init 会无条件调用 get_type_description_func 与其 sources 版本
// （rcl/publisher.c:134-135），传给 rcl_node_type_cache_register_type，
// 而后者对入参做非空校验。因此这里也必须给出非空对象。
//
// 诚实性说明（重要）：
//   真实生成代码的描述直接来自 .idl/.msg 源码，是自洽的真实描述。
//   我们的类型源文件是 .proto，不存在对应的 ROS IDL 源码，因此使用官方
//   为此预留的 encoding = "dynamic"
//   （原文：a type created programmatically by a user, thus having no source）。
//   TypeDescription 内容则如实反映**线上格式**（单个 uint8[] data），
//   不冒充任何 .msg 类型。
template<typename ProtoT>
const rosidl_runtime_c__type_description__IndividualTypeDescription &
GetSyntheticIndividualDescription()
{
  static const std::string full_name = ProtoT::descriptor()->full_name();

  // FieldType: 无界 sequence<uint8>
  static rosidl_runtime_c__type_description__FieldType data_type{};
  static rosidl_runtime_c__type_description__Field data_field{};
  static rosidl_runtime_c__type_description__Field__Sequence fields{};
  static rosidl_runtime_c__type_description__IndividualTypeDescription desc{};
  static bool initialized = false;
  if (!initialized) {
    initialized = true;

    rosidl_runtime_c__String__init(&desc.type_name);
    rosidl_runtime_c__String__assign(&desc.type_name, full_name.c_str());

    data_type.type_id = 3;  // FIELD_TYPE_UINT8
    data_type.capacity = 0;
    data_type.string_capacity = 0;
    rosidl_runtime_c__String__init(&data_type.nested_type_name);

    rosidl_runtime_c__String__init(&data_field.name);
    rosidl_runtime_c__String__assign(&data_field.name, "data");
    data_field.type = data_type;
    rosidl_runtime_c__String__init(&data_field.default_value);

    fields.data = &data_field;
    fields.size = 1;
    fields.capacity = 1;

    desc.fields = fields;
  }
  return desc;
}

template<typename ProtoT>
const rosidl_runtime_c__type_description__TypeDescription * GetProtobufTypeDescriptionFunc(
  const rosidl_message_type_support_t * /*handle*/)
{
  static const rosidl_runtime_c__type_description__TypeDescription description = []() {
      rosidl_runtime_c__type_description__TypeDescription out{};
      out.type_description = GetSyntheticIndividualDescription<ProtoT>();
      out.referenced_type_descriptions.data = nullptr;
      out.referenced_type_descriptions.size = 0;
      out.referenced_type_descriptions.capacity = 0;
      return out;
    }();
  return &description;
}

template<typename ProtoT>
const rosidl_runtime_c__type_description__TypeSource__Sequence *
GetProtobufTypeDescriptionSourcesFunc(const rosidl_message_type_support_t * /*handle*/)
{
  static const rosidl_runtime_c__type_description__TypeSource__Sequence sources = []() {
      static rosidl_runtime_c__type_description__TypeSource source{};
      rosidl_runtime_c__String__init(&source.type_name);
      rosidl_runtime_c__String__assign(
        &source.type_name, ProtoT::descriptor()->full_name().c_str());
      rosidl_runtime_c__String__init(&source.encoding);
      // 官方预留取值：由用户在代码中创建、没有 ROS IDL 源文件。
      rosidl_runtime_c__String__assign(&source.encoding, "dynamic");
      // encoding == "dynamic" 时官方要求该字段为空
      // （见 type_source__struct.h 注释）。
      rosidl_runtime_c__String__init(&source.raw_file_contents);

      rosidl_runtime_c__type_description__TypeSource__Sequence seq{};
      seq.data = &source;
      seq.size = 1;
      seq.capacity = 1;
      return seq;
    }();
  return &sources;
}

// ---------------------------------------------------------------------------
// 合成 introspection
// ---------------------------------------------------------------------------
// 只为满足 Jazzy rmw_fastrtps 的强制类型注册检查。它就是 register_type_object
// 的第一道门（TypeSupport_impl.cpp:612）：
//
//   const rosidl_message_type_support_t * type_support_intro =
//       get_type_support_introspection(type_supports);
//   if (!type_support_intro) { return false; }
//
// 为什么这个描述是**准确**的而不是伪造：本 typesupport 的线上格式确实是
// 单个 sequence<uint8>——uint32 长度前缀 + protobuf 原始字节，因此用
// 「一个 uint8[] data 成员」来描述它，与真实 wire format 逐字节一致；
// 别的 DDS 参与者按这个 type object 解码，拿到的就是那串字节，不会错位。
//
// 必须强调它只描述**线上格式**，不描述**内存布局**：offset_ 与数组访问
// 函数指针在注册路径上全程不会被读取（见 TypeSupport_impl.cpp 的
// GetTypeIdentifier），所以这里留空是安全的。
//
// 注意必须是模板：每个 protobuf 类型需要各自独立的一份 static，否则多个
// 类型会共享第一次调用初始化出来的成员表，导致类型名张冠李戴。
template<typename ProtoT>
const rosidl_message_type_support_t & GetSyntheticIntrospection()
{
  static const std::string full_name = ProtoT::descriptor()->full_name();
  static const std::string message_namespace = ProtoNamespace(full_name);
  static const std::string message_name = ProtoName(full_name);

  static const rosidl_typesupport_introspection_cpp::MessageMember member{
    "data",                                                // name
    rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,        // string upper bound
    nullptr,  // members of sub message
    false,    // is key
    true,     // is array
    0,        // array size（0 且非 upper bound -> 走 sequence 标识）
    false,    // is upper bound
    0,        // bytes offset in struct（注册路径不读）
    nullptr,  // default value
    nullptr, nullptr, nullptr, nullptr, nullptr, nullptr  // 访问函数（注册路径不读）
  };

  static const rosidl_typesupport_introspection_cpp::MessageMembers members{
    message_namespace.c_str(),
    message_name.c_str(),
    1,               // member count
    sizeof(ProtoT),  // size of
    false,           // has any key member
    &member,         // message members
    nullptr,         // init function
    nullptr          // fini function
  };

  static const rosidl_message_type_support_t introspection{
    rosidl_typesupport_introspection_cpp::typesupport_identifier,
    &members,
    nullptr,  // func
    // get_type_hash_func 不能为空：rmw/rcl 会无条件调用它。
    &GetProtobufTypeHashFunc<ProtoT>,
    // 这两个也不能为空：rcl_publisher_init 无条件调用，下游做非空校验。
    &GetProtobufTypeDescriptionFunc<ProtoT>,
    &GetProtobufTypeDescriptionSourcesFunc<ProtoT>
  };

  return introspection;
}

}  // namespace detail

/// 取得（并缓存）某个 protobuf 类型的 typesupport handle。
template<typename ProtoT>
const rosidl_message_type_support_t & GetProtobufTypeSupport()
{
  static_assert(
    IsProtobufMessage<ProtoT>::value,
    "ProtoT must derive from google::protobuf::Message");

  static const std::string full_name = ProtoT::descriptor()->full_name();
  static const std::string message_namespace = detail::ProtoNamespace(full_name);
  static const std::string message_name = detail::ProtoName(full_name);

  static message_type_support_callbacks_t callbacks{};
  callbacks.message_namespace_ = message_namespace.c_str();
  callbacks.message_name_ = message_name.c_str();

  // 序列化后的总大小 = 4 字节长度前缀 + protobuf 字节数。
  callbacks.get_serialized_size = [](const void * untyped) -> uint32_t {
      const auto * msg = static_cast<const ProtoT *>(untyped);
      const std::size_t n = msg->ByteSizeLong();
      if (n > static_cast<std::size_t>(std::numeric_limits<uint32_t>::max()) - 4u) {
        return 0u;
      }
      return static_cast<uint32_t>(4u + n);
    };

  callbacks.cdr_serialize = [](const void * untyped, eprosima::fastcdr::Cdr & cdr) -> bool {
      const auto * msg = static_cast<const ProtoT *>(untyped);
      std::string buf;
      if (!msg->SerializeToString(&buf)) {
        return false;
      }
      const auto n = static_cast<uint32_t>(buf.size());
      cdr << n;
      if (n > 0) {
        cdr.serialize_array(reinterpret_cast<const uint8_t *>(buf.data()), n);
      }
      return true;
    };

  callbacks.cdr_deserialize = [](eprosima::fastcdr::Cdr & cdr, void * untyped) -> bool {
      auto * msg = static_cast<ProtoT *>(untyped);
      uint32_t n = 0;
      cdr >> n;
      if (n == 0) {
        msg->Clear();
        return true;
      }
      std::string buf;
      buf.resize(n);
      cdr.deserialize_array(reinterpret_cast<uint8_t *>(&buf[0]), n);
      return msg->ParseFromArray(buf.data(), static_cast<int>(n));
    };

  callbacks.max_serialized_size = [](char & bounds_info) -> size_t {
      bounds_info = ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
      return 0u;
    };

  static rosidl_message_type_support_t ts{};
  ts.typesupport_identifier = rosidl_typesupport_fastrtps_cpp::typesupport_identifier;
  ts.data = &callbacks;
  // rmw 读取 type hash 用的是**传进来的这个顶层 handle**，不是 introspection
  // handle（见 publisher.cpp:260 / subscription.cpp:388），所以这里也必须设。
  ts.get_type_hash_func = &detail::GetProtobufTypeHashFunc<ProtoT>;
  // rcl_publisher_init 对这三个函数同样是无条件调用，缺一即崩。
  ts.get_type_description_func = &detail::GetProtobufTypeDescriptionFunc<ProtoT>;
  ts.get_type_description_sources_func =
    &detail::GetProtobufTypeDescriptionSourcesFunc<ProtoT>;
  ts.func = [](const rosidl_message_type_support_t * handle, const char * id)
    -> const rosidl_message_type_support_t * {
      if (!handle || !id) {
        return nullptr;
      }
      if (std::strcmp(id, rosidl_typesupport_fastrtps_cpp::typesupport_identifier) == 0) {
        return handle;
      }
      // Jazzy 的 rmw_fastrtps 在建 pub/sub 时会强制要求能派生出 introspection
      // handle，否则 register_type_object 直接失败。只回答 introspection_cpp，
      // 对 introspection_c 返回 nullptr，让 rmw 走它自己的 cpp 回退分支。
      if (std::strcmp(id, rosidl_typesupport_introspection_cpp::typesupport_identifier) == 0) {
        return &detail::GetSyntheticIntrospection<ProtoT>();
      }
      return nullptr;
    };

  return ts;
}

}  // namespace amr_dispatcher_proto

#endif  // AMR_DISPATCHER_PROTO__PROTOBUF_TYPESUPPORT_HPP_
