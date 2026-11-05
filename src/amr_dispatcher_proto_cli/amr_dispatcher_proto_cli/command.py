"""`ros2 amr` 命令组。

结构说明：
  `ros2 amr` 是一个顶层命令（注册在 `ros2cli.command` 组），
  其下的 `echo` 是一个 verb（注册在自建的 `amr.verb` 组）。

为什么用 add_subparsers_on_demand？
  这是 ros2cli 官方推荐的惰性加载方式：只在用户真正输入
  `ros2 amr echo` 时才 import 该 verb 模块。对我们尤其重要——
  echo 模块会 import protobuf 运行时，若在 `ros2 --help` 时加载，
  没装 protobuf 的环境会直接报错。

为什么不直接扩展 `ros2 topic echo`？
  见 echo.py 模块文档：echo 的解析路径依赖 Python 消息类，
  protobuf 类型无法通过那条路径。扩展成独立命令比伪造一个
  假的 Python 消息类更诚实，也更容易排错。
"""

from ros2cli.command import add_subparsers_on_demand
from ros2cli.command import CommandExtension


class AmrCommand(CommandExtension):
    """amr_dispatcher 的 proto 工具集。"""

    def add_arguments(self, parser, cli_name, *, argv=None):
        self._parser = parser
        add_subparsers_on_demand(
            parser,
            cli_name,
            'verb',
            'amr.verb',
            required=False,
            argv=argv,
        )

    def main(self, *, parser, args):
        # add_subparsers_on_demand 把选中的扩展实例存到 dest 指定的属性，
        # 也就是 args.verb（不是 args._extension）。
        extension = getattr(args, 'verb', None)
        if extension is None or isinstance(extension, str):
            parser.print_help()
            return 1
        # 自定义 VerbExtension 的签名是 main(self, *, args)，
        # 与 ros2cli CommandExtension 的 main(self, *, parser, args) 不同。
        return extension.main(args=args)
