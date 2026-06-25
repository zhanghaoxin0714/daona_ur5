import time
from oru_threads import OruSequenceThread
import numpy as np



_active_thread = None

def run_sequence_thread(controller, poses, addLogs, desc, speed=0.02, acc=0.005):
    """
    用 OruSequenceThread 在后台执行一串 moveL，避免卡 UI
    controller: RobotController
    poses: [pose1, pose2, ...]，每个 pose 是 6维数组/列表
    addLogs: main.py 里的 self.addLogs
    """
    global _active_thread

    if controller is None:
        addLogs('【WARNING】请先连接机械臂')
        return

    # 关键：保存线程引用，避免线程对象被回收
    thread = OruSequenceThread(
        controller=controller,
        poses=poses,
        speed=speed,
        acc=acc,
        desc=desc
    )
    thread.log.connect(addLogs)

    def _on_finished(ok, msg):
        global _active_thread
        _active_thread = None
        if ok:
            addLogs(f'【INFO】{msg}')
        else:
            addLogs(f'【ERROR】{msg}')

    thread.finished.connect(_on_finished)

    _active_thread = thread
    thread.start()


def prepare_passive_side_grab(controller, addLogs):
    """
    准备抓取被动端 的具体逻辑都写在这里
    """
    # 1. 检查是否连接机械臂
    if controller is None:
        addLogs('【WARNING】请先连接机械臂，再执行“准备抓取被动端”')
        return

    target_pose = np.array([
        0.2133111250460318, 0.334602564497267, 0.5552137228453256, -0.7864963433040948, -3.0300569523720235, 0.017459805936228753
    ])
    try:
        run_sequence_thread(
            controller=controller,
            poses=[target_pose],
            addLogs=addLogs,
            desc='准备抓取被动端',
            speed=0.02,
            acc=0.005
        )
        addLogs('【INFO】已移动到“准备抓取被动端”预设位姿')
    except Exception as e:
        addLogs(f'【ERROR】准备抓取被动端失败: {e}')



def grab_passive_side(controller, addLogs):
    """
    抓取被动端：移动到抓取位姿
    """
    if controller is None:
        addLogs('【WARNING】请先连接机械臂，再执行“抓取被动端”')
        return

    target_pose = np.array([
        0.2133233518516297, 0.33461211904352994, 0.4951807905436903, -0.7866014638808853, -3.0300756890808906, 0.01757711564872995
    ])

    try:
        run_sequence_thread(
            controller=controller,
            poses=[target_pose],
            addLogs=addLogs,
            desc='抓取被动端',
            speed=0.02,
            acc=0.005
        )
        addLogs('【INFO】已移动到“抓取被动端”预设位姿')
    except Exception as e:
        addLogs(f'【ERROR】抓取被动端失败: {e}')


def passive_side_insert(controller, addLogs):
    """
    被动端插入：依次经过三个插入相关的位姿
    先假设当前已经在合适的起始位置（你说的“随机到一个地方”），
    然后按 位置1 -> 位置2 -> 位置3 依次执行 moveL。
    """
    if controller is None:
        addLogs('【WARNING】请先连接机械臂，再执行“被动端插入”')
        return

    # 三个关键插入位姿
    # pose1 = np.array([
    #     0.31332845981067375,
    #     0.25130462791186388,
    #     0.5637755864156422,
    #     -0.7444122826055278,
    #     -3.0467951392064285,
    #     0.006670156096827588,
    # ])

    pose2 = np.array([
        0.2332354574494888, 0.3341767396922059, 0.5291662934350275, -0.7852515730067023, -3.030130102158652,0.008267247219028152
    ])

    pose3 = np.array([
        0.21330101583879477, 0.33462040403154036, 0.5171480264001627, -0.7852169261660363, -3.0301521181220483,0.008276529832622252
    ])


    poses = [pose2,pose3]

    try:
        run_sequence_thread(
            controller=controller,
            poses=poses,
            addLogs=addLogs,
            desc='被动端插入',
            speed=0.02,
            acc=0.005
        )
        addLogs('【INFO】被动端插入动作完成')
    except Exception as e:
        addLogs(f'【ERROR】被动端插入过程中出错: {e}')

def a_yelu(controller, addLogs):
    """
    被动端插入：依次经过三个插入相关的位姿
    先假设当前已经在合适的起始位置（你说的“随机到一个地方”），
    然后按 位置1 -> 位置2 -> 位置3 依次执行 moveL。
    """
    if controller is None:
        addLogs('【WARNING】请先连接机械臂，再执行“螺丝A移动至液路连接器”')
        return

    # 三个关键插入位姿
    pose1 = np.array([
        0.43045196249848755, 0.2636190063433364, 0.704438622842636, 1.5476617572570088, 2.7192408066293488, 0.2531102053108356
    ])

    pose2 = np.array([
        0.36157721516508806, 0.17022121637153145, 0.7190759274770605, -1.5648864865663505, -2.681439062186884, 3.164594981313485e-05
    ])

    pose3 = np.array([
        0.2930640603094025, 0.24781565320872986, 0.6871538581789174, -0.7100186240840267, -3.052373089089492, -0.010927935563458361
    ])


    poses = [pose1, pose2, pose3]

    try:
        run_sequence_thread(
            controller=controller,
            poses=poses,
            addLogs=addLogs,
            desc='移动至液路连接器',
            speed=0.035,
            acc=0.01
        )
    except Exception as e:
        addLogs(f'【ERROR】移动至液路连接器过程中出错: {e}')


def yelu_dianlu(controller, addLogs):
    """
    被动端插入：依次经过三个插入相关的位姿
    先假设当前已经在合适的起始位置（你说的“随机到一个地方”），
    然后按 位置1 -> 位置2 -> 位置3 依次执行 moveL。
    """
    if controller is None:
        addLogs('【WARNING】请先连接机械臂，再执行“被动端插入”')
        return

    # 三个关键插入位姿
    pose1 = np.array([
        0.3024547225296936, 0.23366697999301062, 0.7278547369019659, -1.5669818040589139, -2.6863855733015884, -0.04946835994571795
    ])

    pose2 = np.array([
        0.3214754857793636, 0.2554492544957924, 0.7659220317255034, 1.544059284438105, 2.637466592951346, 0.596526830933488
    ])

    pose3 = np.array([
        0.11250831633465483, 0.38307383609821927, 0.7716565735071484, -0.6492970295197104, -2.9206024392142065, -0.5839729829566056
    ])

    pose4 = np.array([
        0.1412465470803059, 0.40166697690411035, 0.7151411690630208, 0.6758395016401174, 3.058881266333711, -6.708300074149331e-05
    ])

    poses = [pose1, pose2, pose3,pose4]

    try:
        run_sequence_thread(
            controller=controller,
            poses=poses,
            addLogs=addLogs,
            desc='移动至电路连接器',
            speed=0.035,
            acc=0.01
        )
        addLogs('【INFO】移动至电路连接器动作完成')
    except Exception as e:
        addLogs(f'【ERROR】移动至电路连接器过程中出错: {e}')


def bei_pian(controller, addLogs):
    if controller is None:
        addLogs('【WARNING】请先连接机械臂，再执行“抓取被动端”')
        return

    target_pose = np.array([
        0.3696905694242858, 0.04640663422096447, 0.5761032406522458, 1.9490305531597896, 2.434374829977438, 0.08304307939838285
    ])

    try:
        run_sequence_thread(
            controller=controller,
            poses=[target_pose],
            addLogs=addLogs,
            desc='被动端偏执',
            speed=0.02,
            acc=0.005
        )
        addLogs('【INFO】已移动到“被动端偏置”预设位姿')
    except Exception as e:
        addLogs(f'【ERROR】被动端偏置移动失败: {e}')

def bei_hui(controller, addLogs):
    if controller is None:
        addLogs('【WARNING】请先连接机械臂，再执行“抓取被动端”')
        return

    target_pose = np.array([
        0.23323482343505691, 0.3341842782732954, 0.5351405403721797, -0.785315555501729, -3.030138286076745, 0.008335803439997165
    ])

    try:
        run_sequence_thread(
            controller=controller,
            poses=[target_pose],
            addLogs=addLogs,
            desc='被动端回调',
            speed=0.02,
            acc=0.005
        )
        addLogs('【INFO】已移动到“被动端回调”预设位姿')
    except Exception as e:
        addLogs(f'【ERROR】被动端回调移动失败: {e}')

def luosi(controller, addLogs):
    if controller is None:
        addLogs('【WARNING】请先连接机械臂，再执行“移动至螺丝刀”')
        return

    target_pose = np.array([
        0.10657164529345907, 0.1531083941807354, 0.7010089945432291, -0.2503308488611855, -2.913555603977324, -1.0139401816166056
    ])

    try:
        run_sequence_thread(
            controller=controller,
            poses=[target_pose],
            addLogs=addLogs,
            desc='移动到螺丝刀',
            speed=0.035,
            acc=0.01
        )
        addLogs('【INFO】已移动到“螺丝刀”预设位姿')
    except Exception as e:
        addLogs(f'【ERROR】移动到螺丝刀失败: {e}')

def luosiA(controller, addLogs):
    if controller is None:
        addLogs('【WARNING】请先连接机械臂，再执行“移动到螺丝A”')
        return


    # 三个关键插入位姿
    # pose1 = np.array([
    #     0.31332845981067375,
    #     0.25130462791186388,
    #     0.5637755864156422,
    #     -0.7444122826055278,
    #     -3.0467951392064285,
    #     0.006670156096827588,
    # ])

    pose2 = np.array([
        0.2699988209568979, 0.49886662486393657, 0.6743079424373085, -0.7814139886629279, -2.94865290046188, -0.4324282271628597
    ])

    pose3 = np.array([
        0.36354971377232737, 0.409878859641586, 0.6424027314385164, 0.7956876472116244, 3.0355581917752676, 0.00789985715993873
    ])

    poses = [pose2, pose3]

    try:
        run_sequence_thread(
            controller=controller,
            poses=poses,
            addLogs=addLogs,
            desc='移动至螺丝A',
            speed=0.03,
            acc=0.01
        )
        addLogs('【INFO】移动至螺丝A动作完成')
    except Exception as e:
        addLogs(f'【ERROR】移动至螺丝A过程中出错: {e}')
