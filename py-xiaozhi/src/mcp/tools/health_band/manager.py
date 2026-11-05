"""Register read-only health-band MCP tools."""

from src.utils.logging_config import get_logger

from .tools import assess, history, latest, status

logger = get_logger(__name__)


class HealthBandToolsManager:
    def init_tools(self, add_tool, PropertyList, Property, PropertyType) -> None:
        device_props = PropertyList([Property("device_mac", PropertyType.STRING, default_value="")])
        add_tool((
            "self.health_band.latest",
            "【手环最新体征】用户询问心率、血氧、体温、电量、最新手环数据时必须调用。返回本地保存的原始测量值、接收时间和数据新鲜度；不作医疗诊断。",
            device_props,
            latest,
        ))
        add_tool((
            "self.health_band.history",
            "【手环体征历史】用户询问最近一段时间的心率、血氧、体温或历史趋势时调用。仅返回本地 SQLite 中的原始记录，不作医疗诊断。",
            PropertyList([
                Property("hours", PropertyType.INTEGER, default_value=1, min_value=1, max_value=744),
                Property("device_mac", PropertyType.STRING, default_value=""),
            ]),
            history,
        ))
        add_tool((
            "self.health_band.status",
            "【手环连接状态】用户询问手环是否在线、有没有上传数据、数据是否过期时必须调用。手环约每30秒上报，默认读取私有规则中的超时值（未配置时为90秒）；传 0 表示使用该默认规则。",
            PropertyList([Property("stale_after_seconds", PropertyType.INTEGER, default_value=0, min_value=0, max_value=3600)]),
            status,
        ))
        add_tool((
            "self.health_band.assess",
            "【手环数据质量】查询最新一条数据的接收新鲜度和字段完整性，仅用于提示数据异常，不能诊断疾病、不能自动控制按摩机器人。",
            PropertyList([Property("stale_after_seconds", PropertyType.INTEGER, default_value=0, min_value=0, max_value=3600)]),
            assess,
        ))
        logger.info("[HealthBandToolsManager] 手环查询工具注册完成")
