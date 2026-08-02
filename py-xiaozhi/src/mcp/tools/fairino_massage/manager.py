"""Register FAIRINO trajectory-based massage tools."""

from src.utils.logging_config import get_logger

from .tools import (
    adjust_force,
    detect_meridian,
    get_status,
    pause_massage,
    resume_massage,
    start_massage,
    start_shun_jin,
    stop_massage,
)

logger = get_logger(__name__)


class FairinoMassageToolsManager:
    def __init__(self) -> None:
        self._initialized = False

    def init_tools(self, add_tool, PropertyList, Property, PropertyType) -> None:
        try:
            detect_props = PropertyList(
                [
                    Property("target", PropertyType.STRING, default_value="back"),
                    Property("display", PropertyType.BOOLEAN, default_value=True),
                    Property("timeout_s", PropertyType.INTEGER, default_value=0),
                    Property("stable_frames", PropertyType.INTEGER, default_value=0),
                ]
            )
            add_tool(
                (
                    "self.fairino_massage.detect",
                    "【FAIRINO经络检测】用户说“检测背部经络、检测膀胱经、检测庞光经、进行背部检测、检测大腿外侧、检测大腿内侧、重新检测轨迹”时必须调用本工具。"
                    "语音播报限制：涉及背部经络时一律口播“庞光经”（读音 pang guang jing），不要使用“膀胱经”或“旁光经”，避免TTS读成 bang guang jing。"
                    "功能：后台启动经络/部位检测，默认打开检测画面，检测稳定后保存轨迹并记录当前会话。"
                    "检测成功保存轨迹后，客户端会自动请求小智调用 self.fairino_massage.status 检查检测状态，并播报检测完成和轨迹保存结果。"
                    "工具返回 success=true 仅表示检测任务已启动；应回复用户正在检测，不要判定为失败。"
                    "用户追问检测好了没有、检测状态、轨迹保存了吗时必须调用 self.fairino_massage.status，不要凭记忆回答。"
                    "target 可取 back(背部庞光经)、leg(大腿外侧)、leg_inner(大腿内侧)。",
                    detect_props,
                    detect_meridian,
                )
            )

            start_props = PropertyList(
                [
                    Property("actions", PropertyType.STRING, default_value="all"),
                    Property("target", PropertyType.STRING, default_value="auto"),
                    Property("trajectory_path", PropertyType.STRING, default_value=""),
                ]
            )
            add_tool(
                (
                    "self.fairino_massage.start",
                    "【FAIRINO开始按摩】用户说“开始按摩、开始全套按摩、开始点筋、开始分筋”时必须调用本工具。"
                    "如果用户明确说“开始顺筋、只做顺筋、只执行顺筋、顺筋测试、检查顺筋效果”，必须优先调用 self.fairino_massage.shunjin，不要用 all。"
                    "不要只用自然语言回复进度；只有工具返回后才能告诉用户是否已启动。"
                    "功能：使用当前已检测保存的轨迹，后台执行点筋/分筋/顺筋。"
                    "actions 可取 all 或 dian_jin,fen_jin,shun_jin 的逗号组合；无 trajectory_path 时使用当前会话轨迹。",
                    start_props,
                    start_massage,
                )
            )

            shun_props = PropertyList(
                [
                    Property("target", PropertyType.STRING, default_value="auto"),
                    Property("trajectory_path", PropertyType.STRING, default_value=""),
                ]
            )
            add_tool(
                (
                    "self.fairino_massage.shunjin",
                    "【FAIRINO只执行顺筋】用户说“开始顺筋、只做顺筋、只执行顺筋、单独顺筋、顺筋测试、检查顺筋、检查顺筋效果、测试顺筋贴合”时必须调用本工具。"
                    "功能：使用当前已检测保存的轨迹，只执行 shun_jin 顺筋动作，不执行点筋和分筋，方便检查顺筋末端是否贴合人体。"
                    "无 trajectory_path 时使用当前会话轨迹；如果没有已保存轨迹，工具会返回失败并提示先检测。",
                    shun_props,
                    start_shun_jin,
                )
            )
            add_tool(
                (
                    "self.fairino_massage.shun_jin",
                    "【FAIRINO只执行顺筋别名】与 self.fairino_massage.shunjin 相同。用户要求只做顺筋或检查顺筋贴合时调用本工具。",
                    shun_props,
                    start_shun_jin,
                )
            )

            adjust_force_props = PropertyList(
                [
                    Property("direction", PropertyType.STRING, default_value="stronger"),
                    Property("delta_n", PropertyType.INTEGER, default_value=0),
                ]
            )
            add_tool(
                (
                    "self.fairino_massage.adjust_force",
                    "【FAIRINO调整按摩力度】用户在按摩已经开始或按摩已经暂停后说“大力一些、加大力度、重一点、用力一点、小力一些、减轻力度、减小力度、轻一点、弱一点”时必须调用本工具。"
                    "不要调用 start/pause/resume/status 代替；运行中本工具会请求当前执行器在最近控制检查周期调整目标力，暂停期间会保存新的目标力并在继续按摩后生效。"
                    "必须严格区分方向：用户说大力、加大、增大、重一点、用力一点时 direction=stronger；用户说小力、减轻、减小、降低、轻一点、弱一点时 direction=softer。"
                    "用户明确说数值时，delta_n 填绝对值正数，方向仍由 direction 决定：例如“减轻10N”必须传 direction=softer, delta_n=10；“增大10N”必须传 direction=stronger, delta_n=10。"
                    "不要因为 delta_n 是正数就理解为增大；不要把减轻/降低/小力传成 stronger。用户只说大力/小力且没说数值时不要填 delta_n，默认每次调整 5N。",
                    adjust_force_props,
                    adjust_force,
                )
            )

            add_tool(
                (
                    "self.fairino_massage.pause",
                    "【FAIRINO暂停按摩】用户明确说“暂停按摩、先停一下、等一下、停一下、暂停机械臂、别动、先暂停”时必须调用本工具。"
                    "不要把暂停理解成继续/恢复，也不要改调用 resume/status/start。"
                    "功能：请求执行器在最近安全检查点暂停，并保存轨迹、点位、机械臂位姿等状态。",
                    PropertyList([]),
                    pause_massage,
                )
            )

            add_tool(
                (
                    "self.fairino_massage.resume",
                    "【FAIRINO继续按摩】仅当用户明确说“继续按摩、恢复按摩、接着按摩、从刚才的位置继续”时调用。"
                    "如果用户说的是暂停/停一下/别动，必须调用 pause，不能调用本工具。"
                    "功能：读取上次暂停状态，从保存的阶段和点位继续执行。",
                    PropertyList([]),
                    resume_massage,
                )
            )

            add_tool(
                (
                    "self.fairino_massage.continue",
                    "【FAIRINO继续按摩别名】用户说“继续、继续按摩、接着按、接着按摩、恢复、恢复按摩、从暂停处继续”时调用。"
                    "这是 resume 的别名，只用于从 paused/pausing 状态继续，不用于开始新的按摩。",
                    PropertyList([]),
                    resume_massage,
                )
            )

            add_tool(
                (
                    "self.fairino_massage.resume_massage",
                    "【FAIRINO恢复按摩别名】用户明确要求恢复暂停的按摩任务时调用。"
                    "如果当前状态是 paused，本工具会从保存的 resume_stage/resume_action/resume_point_index 继续。",
                    PropertyList([]),
                    resume_massage,
                )
            )

            add_tool(
                (
                    "self.fairino_massage.stop",
                    "【FAIRINO停止按摩】用户明确说停止按摩、结束按摩、停止机械臂按摩、立即停止、停下来时必须立即调用本工具。"
                    "不得只用自然语言回复已停止，也不得改调用 pause/resume/status/start。"
                    "功能：请求执行器停止当前按摩任务，先退回当前按摩点贴近前的悬空位，再回到已记录的起始位置，并保存停止状态。紧急情况仍应使用物理急停。",
                    PropertyList([]),
                    stop_massage,
                )
            )

            add_tool(
                (
                    "self.fairino_massage.status",
                    "【FAIRINO检测/按摩状态】用户询问“检测状态、检测好了没有、轨迹保存了吗、按摩状态、现在到哪里了、是否暂停、当前轨迹”时必须调用本工具。"
                    "必须以工具返回的 status/stage/current_point_index/progress 为准，不能编造进度。"
                    "语音播报限制：如果涉及背部经络，一律口播“庞光经”（读音 pang guang jing），不要使用“膀胱经”或“旁光经”。"
                    "功能：返回当前会话、轨迹、动作阶段、点位进度、机械臂位姿和状态文件路径。",
                    PropertyList([]),
                    get_status,
                )
            )

            self._initialized = True
            logger.info("[FairinoMassageManager] FAIRINO按摩工具注册完成")
        except Exception as exc:
            logger.error(f"[FairinoMassageManager] 注册失败: {exc}", exc_info=True)
            raise
