"""`ros2 amr echo` 的 verb 实现。"""

from .verb import VerbExtension

from .echo import echo_topic


class EchoVerb(VerbExtension):
    """打印 protobuf 编码的 ROS 话题内容。"""

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            'topic',
            nargs='?',
            help='话题名，例如 /chassis/link_health_proto（--list-types 时可省略）',
        )
        parser.add_argument(
            'type_name',
            nargs='?',
            help=(
                'protobuf 完整类型名，例如 '
                'amr_dispatcher.proto.ChassisLinkHealth。'
                '可用 --list-types 查看全部可用类型。'
            ),
        )
        parser.add_argument(
            '--count', '-c',
            type=int,
            default=0,
            help='收到 N 条后退出；0 表示持续运行（默认 0）',
        )
        parser.add_argument(
            '--once',
            action='store_true',
            help='等价于 --count 1',
        )
        parser.add_argument(
            '--reliability',
            choices=('reliable', 'best_effort'),
            default='reliable',
            help='QoS 可靠性（默认 reliable）',
        )
        parser.add_argument(
            '--durability',
            choices=('volatile', 'transient_local'),
            default='volatile',
            help='QoS 持久性（默认 volatile）',
        )
        parser.add_argument(
            '--depth',
            type=int,
            default=10,
            help='QoS 队列深度（默认 10）',
        )
        parser.add_argument(
            '--list-types',
            action='store_true',
            help='列出可解析的 protobuf 类型后退出（此时 topic/type_name 可省略）',
        )

    def main(self, *, args):
        if args.list_types:
            from .descriptor_pool import ProtoSchemaError
            from .descriptor_pool import registry

            try:
                names = registry.message_names()
            except ProtoSchemaError as exc:
                print(f'错误: {exc}')
                return 2

            print('可解析的 protobuf 类型：')
            for name in names:
                print(f'  {name}')
            return 0

        count = 1 if args.once else args.count

        if not args.topic or not args.type_name:
            print('错误: 需要同时提供 topic 和 type_name（或使用 --list-types）')
            return 2

        return echo_topic(
            topic=args.topic,
            type_name=args.type_name,
            count=count,
            reliability=args.reliability,
            durability=args.durability,
            depth=args.depth,
        )
