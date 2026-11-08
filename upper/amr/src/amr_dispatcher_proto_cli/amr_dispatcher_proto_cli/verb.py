"""`amr.verb` 扩展点的基类。

对应 ros2topic 的 `ros2topic.verb:VerbExtension`。
自定义扩展点必须提供一个 VerbExtension 子类作为基类，
entry point `ros2cli.extension_point` 指向它。
"""

from ros2cli.plugin_system import satisfies_version

# 自定义扩展点版本。改动 verb 接口语义时要升这个版本号。
EXTENSION_POINT_VERSION = '0.1'
PLUGIN_SYSTEM_VERSION = '0.1'


class VerbExtension:
    """
    `ros2 amr` 子命令的扩展点基类。

    必须定义：
      * `NAME`（由 entry point 名自动设置）
    必须实现：
      * `main`
    可选：
      * `add_arguments`
    """

    NAME = None
    EXTENSION_POINT_VERSION = EXTENSION_POINT_VERSION

    def __init__(self):
        super(VerbExtension, self).__init__()
        satisfies_version(PLUGIN_SYSTEM_VERSION, '^0.1')

    def add_arguments(self, parser, cli_name):
        pass

    def main(self, *, args):
        raise NotImplementedError()
