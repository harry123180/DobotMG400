import socket
import numpy as np
import os
import re
import json
import threading
import time
from time import sleep
import requests

alarmControllerFile = "files/alarmController.json"
alarmServoFile = "files/alarmServo.json"

#brief dobot_v4_api:CRA\E6\CRAF\NovaLite
#author futingxing
#date 2025-12-15

# Port Feedback
MyType = np.dtype([('len', np.uint16,),
                   ('reserve', np.byte, (6, )),
                   ('DigitalInputs', np.uint64,),
                   ('DigitalOutputs', np.uint64,),
                   ('RobotMode', np.uint64,),
                   ('TimeStamp', np.uint64,),
                   ('RunTime', np.uint64,),
                   ('TestValue', np.uint64,),
                   ('reserve2', np.byte, (8, )),
                   ('SpeedScaling', np.float64,),
                   ('reserve3', np.byte, (16, )),
                   ('VRobot', np.float64, ),      
                   ('IRobot', np.float64,),
                   ('ProgramState', np.float64,),
                   ('SafetyOIn', np.uint16,),
                   ('SafetyOOut', np.uint16,),
                   ('reserve4', np.byte, (76, )),
                   ('QTarget', np.float64, (6, )),
                   ('QDTarget', np.float64, (6, )),
                   ('QDDTarget', np.float64, (6, )),
                   ('ITarget', np.float64, (6, )),
                   ('MTarget', np.float64, (6, )),
                   ('QActual', np.float64, (6, )),
                   ('QDActual', np.float64, (6, )),
                   ('IActual', np.float64, (6, )),
                   ('ActualTCPForce', np.float64, (6, )),
                   ('ToolVectorActual', np.float64, (6, )),
                   ('TCPSpeedActual', np.float64, (6, )),
                   ('TCPForce', np.float64, (6, )),
                   ('ToolVectorTarget', np.float64, (6, )),
                   ('TCPSpeedTarget', np.float64, (6, )),
                   ('MotorTemperatures', np.float64, (6, )),
                   ('JointModes', np.float64, (6, )),
                   ('VActual', np.float64, (6, )),
                   ('HandType', np.byte, (4, )),
                   ('User', np.byte,),
                   ('Tool', np.byte,),
                   ('RunQueuedCmd', np.byte,),
                   ('PauseCmdFlag', np.byte,),
                   ('VelocityRatio', np.byte,),
                   ('AccelerationRatio', np.byte,),
                   ('reserve5', np.byte, ),
                   ('XYZVelocityRatio', np.byte,),
                   ('RVelocityRatio', np.byte,),
                   ('XYZAccelerationRatio', np.byte,),
                   ('RAccelerationRatio', np.byte,),
                   ('reserve6', np.byte,(2,)),
                   ('BrakeStatus', np.byte,),
                   ('EnableStatus', np.byte,),
                   ('DragStatus', np.byte,),
                   ('RunningStatus', np.byte,),
                   ('ErrorStatus', np.byte,),
                   ('JogStatusCR', np.byte,),   
                   ('CRRobotType', np.byte,),
                   ('DragButtonSignal', np.byte,),
                   ('EnableButtonSignal', np.byte,),
                   ('RecordButtonSignal', np.byte,),
                   ('ReappearButtonSignal', np.byte,),
                   ('JawButtonSignal', np.byte,),
                   ('SixForceOnline', np.byte,),
                   ('CollisionState', np.byte,),
                   ('ArmApproachState', np.byte,),
                   ('J4ApproachState', np.byte,),
                   ('J5ApproachState', np.byte,),
                   ('J6ApproachState', np.byte,),
                   ('reserve7', np.byte, (61, )),
                   ('VibrationDisZ', np.float64,),
                   ('CurrentCommandId', np.uint64,),
                   ('MActual', np.float64, (6, )),
                   ('Load', np.float64,),
                   ('CenterX', np.float64,),
                   ('CenterY', np.float64,),
                   ('CenterZ', np.float64,),
                   ('UserValue[6]', np.float64, (6, )),
                   ('ToolValue[6]', np.float64, (6, )),
                   ('reserve8', np.byte, (8, )),
                   ('SixForceValue', np.float64, (6, )),
                   ('TargetQuaternion', np.float64, (4, )),
                   ('ActualQuaternion', np.float64, (4, )),
                   ('AutoManualMode', np.uint16, ),
                   ('ExportStatus', np.uint16, ),
                   ('SafetyState', np.byte, ),
                   ('reserve9', np.byte,(19,))
                   ])

# 讀取控制器和伺服告警檔案
# Read controller and servo alarm files
def alarmAlarmJsonFile():
    currrntDirectory = os.path.dirname(__file__)
    jsonContrellorPath = os.path.join(currrntDirectory, alarmControllerFile)
    jsonServoPath = os.path.join(currrntDirectory, alarmServoFile)

    with open(jsonContrellorPath, encoding='utf-8') as f:
        dataController = json.load(f)
    with open(jsonServoPath, encoding='utf-8') as f:
        dataServo = json.load(f)
    return dataController, dataServo

# TCP 通訊介面類別
# TCP communication interface


class DobotApi:
    def __init__(self, ip, port, *args):
        self.ip = ip
        self.port = port
        self.socket_dobot = 0
        self.__globalLock = threading.Lock()
        if args:
            self.text_log = args[0]

        if self.port == 29999 or self.port == 30004 or self.port == 30005:
            try:
                self.socket_dobot = socket.socket()
                self.socket_dobot.connect((self.ip, self.port))
                self.socket_dobot.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 144000)
            except socket.error:
                print(socket.error)

        else:
            print(f"Connect to dashboard server need use port {self.port} !")

    def log(self, text):
        if self.text_log:
            print(text)

    def send_data(self, string):
       # self.log(f"Send to {self.ip}:{self.port}: {string}")
        try:
            self.socket_dobot.send(str.encode(string, 'utf-8'))
        except Exception as e:
            print(e)
            while True:
                try:
                    self.socket_dobot = self.reConnect(self.ip, self.port)
                    self.socket_dobot.send(str.encode(string, 'utf-8'))
                    break
                except Exception:
                    sleep(1)

    def wait_reply(self):
        """
        Read the return value
        """
        data = ""
        try:
            data = self.socket_dobot.recv(1024)
        except Exception as e:
            print(e)
            self.socket_dobot = self.reConnect(self.ip, self.port)

        finally:
            if len(data) == 0:
                data_str = data
            else:
                data_str = str(data, encoding="utf-8")
            # self.log(f'Receive from {self.ip}:{self.port}: {data_str}')
            return data_str

    def close(self):
        """
        Close the port
        """
        if (self.socket_dobot != 0):
            try:
                self.socket_dobot.shutdown(socket.SHUT_RDWR)
                self.socket_dobot.close()
            except socket.error as e:
                print(f"Error while closing socket: {e}")

    def sendRecvMsg(self, string):
        """
        send-recv Sync
        """
        with self.__globalLock:
            self.send_data(string)
            recvData = self.wait_reply()
            return recvData

    def __del__(self):
        self.close()

    def reConnect(self, ip, port):
        while True:
            try:
                socket_dobot = socket.socket()
                socket_dobot.connect((ip, port))
                break
            except Exception:
                sleep(1)
        return socket_dobot

# 控制及運動指令介面類別
# Control and motion command interface


class DobotApiDashboard(DobotApi):

    def __init__(self, ip, port, *args):
        super().__init__(ip, port, *args)

    def _fmt(self, v):
        if isinstance(v, (list, tuple)):
            return "{" + ",".join([self._fmt(x) for x in v]) + "}"
        if isinstance(v, float):
            return ("{:f}".format(v))
        if isinstance(v, int):
            return ("{:d}".format(v))
        return str(v)

    def _build_cmd(self, name, *args, **kwargs):
        parts = []
        for a in args:
            parts.append(self._fmt(a))
        for k, v in kwargs.items():
            parts.append(f"{k}={self._fmt(v)}")
        return f"{name}(" + ",".join(parts) + ")"

    def EnableRobot(self, load=0.0, centerX=0.0, centerY=0.0, centerZ=0.0, isCheck=-1,):
        """
            可選參數
            參數名 類型 說明
            load double
            設定負載重量，取值範圍不能超過各個型號機器人的負載範圍。單位：kg
            centerX double X 方向偏心距離。取值範圍：-999~ 999，單位：mm
            centerY double Y 方向偏心距離。取值範圍：-999~ 999，單位：mm
            centerZ double Z 方向偏心距離。取值範圍：-999~ 999，單位：mm
            isCheck int    是否檢查負載。1 表示檢查，0 表示不檢查。如果設定為 1，則機械臂
            使能後會檢查實際負載是否和設定負載一致，如果不一致會自動下使
            能。預設值為 0
            可攜帶的參數數量如下：
            0：不攜帶參數，表示使能時不設定負載重量和偏心參數。
            1：攜帶一個參數，該參數表示負載重量。
            4：攜帶四個參數，分別表示負載重量和偏心參數。
            5：攜帶五個參數，分別表示負載重量、偏心參數和是否檢查負載。
                """
        """
            Optional parameter
            Parameter name     Type     Description
            load     double     Load weight. The value range should not exceed the load range of corresponding robot models. Unit: kg.
            centerX     double     X-direction eccentric distance. Range: -999 – 999, unit: mm.
            centerY     double     Y-direction eccentric distance. Range: -999 – 999, unit: mm.
            centerZ     double     Z-direction eccentric distance. Range: -999 – 999, unit: mm.
            isCheck     int     Check the load or not. 1: check, 0: not check. If set to 1, the robot arm will check whether the actual load is the same as the set load after it is enabled, and if not, it will be automatically disabled. 0 by default.
            The number of parameters that can be contained is as follows:
            0: no parameter (not set load weight and eccentric parameters when enabling the robot).
            1: one parameter (load weight).
            4: four parameters (load weight and eccentric parameters).
            5: five parameters (load weight, eccentric parameters, check the load or not).
                """
        string = 'EnableRobot('
        if load != 0:
            string = string + "{:f}".format(load)
            if centerX != 0 or centerY != 0 or centerZ != 0:
                string = string + ",{:f},{:f},{:f}".format(
                    centerX, centerY, centerZ)
                if isCheck != -1:
                    string = string + ",{:d}".format(isCheck)
        string = string + ')'
        return self.sendRecvMsg(string)

    def DisableRobot(self):
        """
        Disabled the robot
        下使能機械臂
        """
        string = "DisableRobot()"
        return self.sendRecvMsg(string)

    def ClearError(self):
        """
        Clear controller alarm information
        Clear the alarms of the robot. After clearing the alarm, you can judge whether the robot is still in the alarm status according to RobotMode.
        Some alarms cannot be cleared unless you resolve the alarm cause or restart the controller.
        清除機器人報警。清除報警後，使用者可以根據 RobotMode 來判斷機器人是否還處於報警狀態。部
        分報警需要解決報警原因或者重啟控制櫃後才能清除。
        """
        string = "ClearError()"
        return self.sendRecvMsg(string)

    def PowerOn(self):
        """
        Powering on the robot
        Note: It takes about 10 seconds for the robot to be enabled after it is powered on.
        """
        string = "PowerOn()"
        return self.sendRecvMsg(string)

    def RunScript(self, project_name):
        """
        Run the script file
        project_name ：Script file name
        """
        string = "RunScript({:s})".format(project_name)
        return self.sendRecvMsg(string)

    def Stop(self):
        """
       停止已下發的運動指令佇列或者 RunScript 指令執行的工程。
       Stop the delivered motion command queue or the RunScript command from running.
        """
        string = "Stop()"
        return self.sendRecvMsg(string)

    def Pause(self):
        """
       暫停已下發的運動指令佇列或者 RunScript 指令執行的工程。
       Pause the delivered motion command queue or the RunScript command from running.
        """
        string = "Pause()"
        return self.sendRecvMsg(string)

    def Continue(self):
        """
       繼續已暫停的運動指令佇列或者 RunScript 指令執行的工程。
       Continue the paused motion command queue or the RunScript command from running.
        """
        string = "Continue()"
        return self.sendRecvMsg(string)

    def EmergencyStop(self, mode):
        """
       緊急停止機械臂。急停後機械臂會下使能並報警，需要鬆開急停、清除報警後才能重新使能。
       必選參數
       參數名 類型 說明
        mode int 急停操作模式。1 表示按下急停，0 表示鬆開急停
       Stop the robot in an emergency. After the emergency stop, the robot arm will be disabled and then alarm. You need to release the emergency stop and clear the alarm to re-enable the robot arm.
       Required parameter
       Parameter name     Type     Description
        mode     int     E-Stop operation mode. 1: press the E-Stop, 0: release the E-Stop.
        """
        string = "EmergencyStop({:d})".format(mode)
        return self.sendRecvMsg(string)

    def BrakeControl(self, axisID, value):
        """
        描述
        控制指定關節的抱閘。機械臂靜止時關節會自動抱閘，如果使用者需進行關節拖拽操作，可開啟抱
        閘，即在機械臂下使能狀態，手動扶住關節後，下發開啟抱閘的指令。
        僅能在機器人下使能時控制關節抱閘，否則 ErrorID 會回傳 -1。
        必選參數
        參數名  類型  說明
        axisID int 關節軸序號，1 表示 J1 軸，2 表示 J2 軸，以此類推
        value int 設定抱閘狀態。0 表示抱閘鎖死（關節不可移動），1 表示鬆開抱閘（關節
        可移動）
        Description
        Control the brake of specified joint. The joints automatically brake when the robot is stationary. If you need to drag the joints, you can switch on the brake,
        i.e. hold the joint manually in the disabled status and deliver the command to switch on the brake.
        Joint brake can be controlled only when the robot arm is disabled, otherwise, Error ID will return -1.
        Required parameter:
        Parameter name     Type     Description
        axisID     int     joint ID, 1: J1, 2: J2, and so on
        Value     int     Set the status of brake. 0: switch off brake (joints cannot be dragged). 1: switch on brake (joints can be dragged).
        """
        string = "BrakeControl({:d},{:d})".format(axisID, value)
        return self.sendRecvMsg(string)

    #####################################################################

    def SpeedFactor(self, speed):
        """
        設定全域速度比例。
           機械臂點動時實際運動加速度/速度比例 = 控制軟體點動設定中的值 x 全域速度比例。
           例：控制軟體設定的關節速度為 12°/s，全域速率為 50%，則實際點動速度為 12°/s x 50% =
           6°/s
           機械臂再現時實際運動加速度/速度比例 = 運動指令可選參數設定的比例 x 控制軟體再現設定
           中的值 x 全域速度比例。
           例：控制軟體設定的座標系速度為 2000mm/s，全域速率為 50%，運動指令設定的速率為
           80%，則實際運動速度為 2000mm/s x 50% x 80% = 800mm/s
        未設定時沿用進入 TCP/IP 控制模式前控制軟體設定的值。
        取值範圍：[1, 100]
        Set the global speed ratio.
           Actual robot acceleration/speed ratio in jogging = value in Jog settings × global speed ratio.
           Example: If the joint speed set in the software is 12°/s and the global speed ratio is 50%, then the actual jog speed is 12°/s x 50% =
           6°/s
           Actual robot acceleration/speed ratio in playback = ratio set in motion command × value in Playback settings
            × global speed ratio.
           Example: If the coordinate system speed set in the software is 2000mm/s, the global speed ratio is 50%, and the speed set in the motion command is
           80%, then the actual speed is 2000mm/s x 50% x 80% = 800mm/s.
        If it is not set, the value set by the software before entering TCP/IP control mode will be adopted.
        Range: [1, 100].
        """
        string = "SpeedFactor({:d})".format(speed)
        return self.sendRecvMsg(string)

    def User(self, index):
        """
        設定全域使用者座標系。使用者下發運動指令時可選擇使用者座標系，如未指定，則會使用全域使用者座標系。
        未設定時預設的全域使用者座標系為使用者座標系 0。
        Set the global tool coordinate system. You can select a tool coordinate system while delivering motion commands. If you do not specify the tool coordinate system, the global tool coordinate system will be used.
        If it is not set, the default global user coordinate system is User coordinate system 0.
        """
        string = "User({:d})".format(index)
        return self.sendRecvMsg(string)

    def SetUser(self, index, table):
        """
        修改指定的使用者座標系。
        必選參數：
        參數名 類型 說明
        index int 使用者座標系索引，取值範圍：[0,9]，座標系 0 初始值為基座標系。
        table string  修改後的使用者座標系，格式為 {x, y, z, rx, ry, rz}，建議使用 CalcUser 指令獲
        取。
        Modify the specified user coordinate system.
        Required parameter:
        Parameter name     Type     Description
        index    int     user coordinate system index, range: [0,9]. The initial value of coordinate system 0 refers to the base coordinate system.
        table    string     user coordinate system after modification (format: {x, y, z, rx, ry, rz}), which is recommended to obtain through "CalcUser" command.
        """
        string = "SetUser({:d},{:s})".format(index, table)
        return self.sendRecvMsg(string)

    def CalcUser(self, index, matrix_direction, table):
        """
        計算使用者座標系。
        必選參數：
        參數名 類型 說明
        index int 使用者座標系索引，取值範圍：[0,9]，座標系 0 初始值為基座標系。
        matrix_direction int  計算的方向。1 表示左乘，即 index 指定的座標系沿基座標系偏轉 table 指定的值；
            0 表示右乘，即 index 指定的座標系沿自己偏轉 table 指定的值。
        table string 使用者座標系偏移值，格式為 {x, y, z, rx, ry, rz}。
        Calculate the user coordinate system.
        Required parameter:
        Parameter name     Type     Description
        Index    int     user coordinate system index, range: [0,9]. The initial value of coordinate system 0 refers to the base coordinate system.
        matrix_direction    int    Calculation method. 1: left multiplication, indicating that the coordinate system specified by "index" deflects the value specified by "table" along the base coordinate system.
            0: right multiplication, indicating that the coordinate system specified by "index" deflects the value specified by "table" along itself.
        table    string     user coordinate system offset (format: {x, y, z, rx, ry, rz}).
        """
        string = "SetUser({:d},{:d},{:s})".format(
            index, matrix_direction, table)
        return self.sendRecvMsg(string)

    def Tool(self, index):
        """
        設定全域工具座標系。使用者下發運動指令時可選擇工具座標系，如未指定，則會使用全域工具座標系。
        未設定時預設的全域工具座標系為工具座標系 0。
        Set the global tool coordinate system. You can select a tool coordinate system while delivering motion commands. If you do not specify the tool coordinate system, the global tool coordinate system will be used.
        If it is not set, the default global tool coordinate system is Tool coordinate system 0.
        """
        string = "Tool({:d})".format(index)
        return self.sendRecvMsg(string)

    def SetTool(self, index, table):
        """
        修改指定的工具座標系。
        必選參數：
        參數名 類型 說明
        index int 工具座標系索引，取值範圍：[0,9]，座標系 0 初始值為法蘭座標系。
        table string  修改後的工具座標系，格式為 {x, y, z, rx, ry, rz}，表示該座標系相對預設工
        具座標系的偏移量。
        Modify the specified tool coordinate system.
        Required parameter:
        Parameter name     Type     Description
        Index    int     tool coordinate system index, range: [0,9]. The initial value of coordinate system 0 refers to the flange coordinate system.
        table    string     tool coordinate system after modification (format: {x, y, z, rx, ry, rz})
        """
        string = "SetTool({:d},{:s})".format(index, table)
        return self.sendRecvMsg(string)

    def CalcTool(self, index, matrix_direction, table):
        """
        計算工具座標系。
        必選參數：
        參數名 類型 說明
        index int  工具座標系索引，取值範圍：[0,9]，座標系 0 初始值為法蘭座標系。
        matrix_direction int 計算的方向。
          1 表示左乘，即 index 指定的座標系沿法蘭座標系偏轉 table 指定的值；
          0 表示右乘，即 index 指定的座標系沿自己偏轉 table 指定的值。
        table string 工具座標系偏移值，格式為 {x, y, z, rx, ry, rz}。
        Calculate the tool coordinate system.
        Required parameter:
        Parameter name     Type     Description
        Index    int     tool coordinate system index, range: [0,9]. The initial value of coordinate system 0 refers to the flange coordinate system.
        matrix_direction    int    Calculation method.
          1: left multiplication, indicating that the coordinate system specified by "index" deflects the value specified by "table" along the flange coordinate system.
          0: right multiplication, indicating that the coordinate system specified by "index" deflects the value specified by "table" along itself.
        table    string     tool coordinate system offset (format: {x, y, z, rx, ry, rz}).
        """
        string = "CalcTool({:d},{:d},{:s})".format(
            index, matrix_direction, table)
        return self.sendRecvMsg(string)

    def SetPayload(self, load=0.0, X=0.0, Y=0.0, Z=0.0, name='F'):
        '''設定機械臂末端負載，支援兩種設定方式。
        方式一：直接設定負載參數
        必選參數 1
        參數名 類型 說明
        load double  設定負載重量，取值範圍不能超過各個型號機器人的負載範圍。單位：kg
        可選參數 1
        參數名 類型 說明
        x double 末端負載 X 軸偏心座標。取值範圍：-500~500。單位：mm
        y double 末端負載 Y 軸偏心座標。取值範圍：-500~500。單位：mm
        z double 末端負載 Z 軸偏心座標。取值範圍：-500~500。單位：mm
        需同時設定或不設定這三個參數。偏心座標為負載（含治具）的質心在預設工具座標系下的座標，
        參考下圖。

        方式二：透過控制軟體儲存的預設負載參數組設定
        必選參數 2
        參數名 類型 說明
        name string 控制軟體儲存的預設負載參數組的名稱
        Set the load of the robot arm.
        Method 1: Set the load parameters directly.
        Required parameter 1
        Parameter name     Type     Description
        load     double     Load weight. The value range should not exceed the load range of corresponding robot models. Unit: kg.
        Optional parameter 1
        Parameter name     Type     Description
        x     double     X-axis eccentric coordinates of the load. Range: -500 – 500. Unit: mm.
        y     double     Y-axis eccentric coordinates of the load. Range: -500 – 500. Unit: mm.
        z     double     Z-axis eccentric coordinates of the load. Range: -500 – 500. Unit: mm.
        The three parameters need to be set or not set at the same time. The eccentric coordinate is the coordinate of the center of mass of the load (including the fixture) under the default tool coordinate system.
        Refer to the figure below.

        Method 2: Set by the preset load parameter group saved by control software
        Required parameter 2
        Parameter name     Type     Description
        name     string     Name of the preset load parameter group saved by control software.
        '''
        string = 'SetPayload('
        if name != 'F':
            string = string + "{:s}".format(name)
        else:
            if load != 0:
                string = string + "{:f}".format(load)
                if X != 0 or Y != 0 or Z != 0:
                    string = string + ",{:f},{:f},{:f}".format(X, Y, Z)
        string = string + ')'
        return self.sendRecvMsg(string)

    def AccJ(self, speed):
        """
        設定關節運動方式的加速度比例。
        未設定時預設值為 100
        Set acceleration ratio of joint motion.
        Defaults to 100 if not set.
        """
        string = "AccJ({:d})".format(speed)
        return self.sendRecvMsg(string)

    def AccL(self, speed):
        """
        設定直線和弧線運動方式的加速度比例。
        未設定時預設值為 100。
        Set acceleration ratio of linear and arc motion.
        Defaults to 100 if not set.
        """
        string = "AccL({:d})".format(speed)
        return self.sendRecvMsg(string)

    def VelJ(self, speed):
        """
        設定關節運動方式的速度比例。
        未設定時預設值為 100。
        Set the speed ratio of joint motion.
        Defaults to 100 if not set.
        """
        string = "VelJ({:d})".format(speed)
        return self.sendRecvMsg(string)

    def VelL(self, speed):
        """
        設定直線和弧線運動方式的速度比例。
        未設定時預設值為 100。
        Set the speed ratio of linear and arc motion.
        Defaults to 100 if not set.
        """
        string = "VelL({:d})".format(speed)
        return self.sendRecvMsg(string)

    def CP(self, ratio):
        """
        設定平滑過渡比例，即機械臂連續運動經過多個點時，經過中間點是以直角方式過渡還是以曲線方式過渡。
        未設定時預設值為 0。
        平滑過渡比例。取值範圍：[0, 100]
        Set the continuous path (CP) ratio, that is, when the robot arm moves continuously via multiple points, whether it transitions at a right angle or in a curved way when passing through the through point.
        Defaults to 0 if not set.
        Continuous path ratio. Range: [0, 100].
        """
        string = "CP({:d})".format(ratio)
        return self.sendRecvMsg(string)

    def SetCollisionLevel(self, level):
        """
        設定碰撞檢測等級。
        未設定時沿用進入 TCP/IP 控制模式前控制軟體設定的值。
        必選參數
        參數名 類型 說明
        level int 碰撞檢測等級，0 表示關閉碰撞檢測，1~5 數字越大靈敏度越高
        Set the collision detection level.
        If it is not set, the value set by the software before entering TCP/IP control mode will be adopted.
        Required parameter:
        Parameter name     Type     Description
        level     int     collision detection level, 0: switch off collision detection, 1 – 5: the larger the number, the higher the sensitivity.
        """
        string = "SetCollisionLevel({:d})".format(level)
        return self.sendRecvMsg(string)

    def SetBackDistance(self, distance):
        """
        設定機械臂檢測到碰撞後原路回退的距離。
        未設定時沿用進入 TCP/IP 控制模式前控制軟體設定的值。
        必選參數：
        參數名 類型 說明
        distance double 碰撞回退的距離，取值範圍：[0,50]，單位：mm
        Set the backoff distance after the robot detects collision.
        If it is not set, the value set by the software before entering TCP/IP control mode will be adopted.
        Required parameter:
        Parameter name     Type     Description
        distance     double     collision backoff distance, range: [0,50], unit: mm.
        """
        string = "SetBackDistance({:d})".format(distance)
        return self.sendRecvMsg(string)

    def SetPostCollisionMode(self, mode):
        """
        設定機械臂檢測到碰撞後進入的狀態。
        未設定時沿用進入 TCP/IP 控制模式前控制軟體設定的值。
        必選參數：
        參數名  類型 說明
        mode int  碰撞後處理方式，0 表示檢測到碰撞後進入停止狀態，1 表示檢測到碰撞後
        進入暫停狀態
        Set the backoff distance after the robot detects collision.
        If it is not set, the value set by the software before entering TCP/IP control mode will be adopted.
        Required parameter:
        Parameter name     Type     Description
        mode     int     post-collision processing mode, 0: enter the stop status after the collision is detected, 1: enter the pause status after the collision is detected
        """
        string = "SetPostCollisionMode({:d})".format(mode)
        return self.sendRecvMsg(string)

    def StartDrag(self):
        """
        機械臂進入拖拽模式。機械臂處於報警狀態下時，無法透過該指令進入拖拽模式。
        The robot arm enters the drag mode. The robot cannot enter the drag mode through this command in error status.
        """
        string = "StartDrag()"
        return self.sendRecvMsg(string)

    def StopDrag(self):
        """
        退出拖拽模式
        The robot arm enters the drag mode. The robot cannot enter the drag mode through this command in error status.
        """
        string = "StopDrag()"
        return self.sendRecvMsg(string)

    def DragSensivity(self, index, value):
        """
        設定拖拽靈敏度。
        未設定時沿用進入 TCP/IP 控制模式前控制軟體設定的值。
        必選參數
        參數名 類型 說明
        index int 軸序號，1~6 分別表示 J1~J6 軸，0 表示所有軸同時設定
        value int 拖拽靈敏度，值越小，拖拽時的阻力越大。取值範圍：[1, 90]
        Set the drag sensitivity.
        If it is not set, the value set by the software before entering TCP/IP control mode will be adopted.
        Required parameter:
        Parameter name     Type     Description
        index     int      axis ID, 1 – 6: J1 – J6, 0: set all axes at the same time.
        value     int     Drag sensitivity. The smaller the value, the greater the force when dragging. Range: [1, 90].
        """
        string = "DragSensivity({:d},{:d})".format(index, value)
        return self.sendRecvMsg(string)

    def EnableSafeSkin(self, status):
        """
        開啟或關閉安全皮膚功能。僅對安裝了安全皮膚的機械臂有效。
        必選參數
        參數名 類型 說明
        status int 電子皮膚功能開關，0 表示關閉，1 表示開啟
        Switch on or off the SafeSkin. Valid only for robot arms equipped with SafeSkin.
        Required parameter:
        Parameter name     Type     Description
        status     int     SafeSkin switch, 0: off, 1: on.
        """
        string = "EnableSafeSkin({:d})".format(status)
        return self.sendRecvMsg(string)

    def SetSafeSkin(self, part, status):
        """
        設定安全皮膚各個部位的靈敏度。僅對安裝了安全皮膚的機械臂有效。
        未設定時沿用進入 TCP/IP 控制模式前控制軟體設定的值。
        必選參數
        參數名 類型 說明
        part int 要設定的部位，3 表示小臂，4~6 分別表示 J4~J6 關節
        status int 靈敏度，0 表示關閉，1 表示 low，2 表示 middle，3 表示 high
        Set the sensitivity for each part of the SafeSkin. Valid only for robot arms equipped with SafeSkin.
        If it is not set, the value set by the software before entering TCP/IP control mode will be adopted.
        Required parameter:
        Parameter name     Type     Description
        part     int     The part to be set. 3: forearm, 4 – 6: J4 – J6
        status     int     sensitivity, 0: off, 1: low, 2: middle, 3: high
        """
        string = "SetSafeSkin({:d},{:d})".format(part, status)
        return self.sendRecvMsg(string)

    def SetSafeWallEnable(self, index, value):
        """
        開啟或關閉指定的安全牆。
        必選參數：
        參數名 類型 說明
        index int 要設定的安全牆索引，需要先在控制軟體中新增對應的安全牆。取值範圍：[1,8]
        value int 安全牆開關，0 表示關閉，1 表示開啟
        Switch on/off the specified safety wall.
        Required parameter:
        Parameter name     Type     Description
        index     int     safety wall index, which needs to be added in the software first. Range: [1.8].
        value      int     SafeSkin switch, 0: off, 1: on.
        """
        string = "SetSafeWallEnable({:d},{:d})".format(index, value)
        return self.sendRecvMsg(string)

    def SetWorkZoneEnable(self, index, value):
        """
        開啟或關閉指定的干涉區。
        必選參數：
        參數名 類型 說明
        index int 要設定的干涉區索引，需要先在控制軟體中新增對應的干涉區。取值範圍：[1,6]
        value int 干涉區開關，0 表示關閉，1 表示開啟
        Switch on/off the specified interference area.
        Required parameter:
        Parameter name     Type     Description
        index     int     interference area index, which needs to be added in the software first. Range: [1.6].
        value      int     interference area switch, 0: off, 1: on.
        """
        string = "SetWorkZoneEnable({:d},{:d})".format(index, value)
        return self.sendRecvMsg(string)

    #########################################################################

    def RobotMode(self):
        """
        獲取機器人當前狀態。
        1 ROBOT_MODE_INIT 初始化狀態
        2 ROBOT_MODE_BRAKE_OPEN 有任意關節的抱閘鬆開
        3 ROBOT_MODE_POWEROFF 機械臂下電狀態
        4 ROBOT_MODE_DISABLED 未使能（無抱閘鬆開）
        5 ROBOT_MODE_ENABLE 使能且空閒
        6 ROBOT_MODE_BACKDRIVE 拖拽模式
        7 ROBOT_MODE_RUNNING 執行狀態（工程，TCP 佇列運動等）
        8 ROBOT_MODE_SINGLE_MOVE 單次運動狀態（點動、RunTo 等）
        9 ROBOT_MODE_ERROR
             有未清除的報警。此狀態優先級最高，無論機械臂
             處於什麼狀態，有報警時都回傳 9
        10 ROBOT_MODE_PAUSE 工程暫停狀態
        11 ROBOT_MODE_COLLISION 碰撞檢測觸發狀態
        Get the current status of the robot.
        1 ROBOT_MODE_INIT  Initialized status
        2 ROBOT_MODE_BRAKE_OPEN  Brake switched on
        3 ROBOT_MODE_POWEROFF  Power-off status
        4 ROBOT_MODE_DISABLED  Disabled (no brake switched on
        5 ROBOT_MODE_ENABLE  Enabled and idle
        6 ROBOT_MODE_BACKDRIVE  Drag mode
        7 ROBOT_MODE_RUNNING  Running status (project, TCP queue)
        8 ROBOT_MODE_SINGLE_MOVE  Single motion status (jog, RunTo)
        9 ROBOT_MODE_ERROR
             There are uncleared alarms. This status has the highest priority. It returns 9 when there is an alarm, regardless of the status of the robot arm.
        10 ROBOT_MODE_PAUSE  Pause status
        11 ROBOT_MODE_COLLISION  Collision status
        """
        string = "RobotMode()"
        return self.sendRecvMsg(string)

    def PositiveKin(self, J1, J2, J3, J4, J5, J6, user=-1, tool=-1):
        """
       描述
       進行正解運算：給定機械臂各關節角度，計算機械臂末端在給定的笛卡爾座標系中的座標值。
       必選參數
       參數名 類型 說明
       J1 double J1軸位置，單位：度
       J2 double J2軸位置，單位：度
       J3 double J3軸位置，單位：度
       J4 double J4軸位置，單位：度
       J5 double J5軸位置，單位：度
       J6 double J6軸位置，單位：度
       可選參數
       參數名 類型 說明
       格式為"user=index"，index為已標定的使用者座標系索引。
       User string 不指定時使用全域使用者座標系。
       Tool string  格式為"tool=index"，index為已標定的工具座標系索引。不指定時使用全域工具座標系。
       Description
       Positive solution. Calculate the coordinates of the end of the robot in the specified Cartesian coordinate system, based on the given angle of each joint.
       Required parameter:
       Parameter name     Type     Description
       J1     double     J1-axis position, unit: °
       J2     double     J2-axis position, unit: °
       J3     double     J3-axis position, unit: °
       J4     double     J4-axis position, unit: °
       J5     double     J5-axis position, unit: °
       J6     double     J6-axis position, unit: °
       Optional parameter:
       Parameter name     Type     Description
       Format: "user=index", index: index of the calibrated user coordinate system.
       User     string     The global user coordinate system will be used if it is not specified.
       Tool     string     Format: "tool=index", index: index of the calibrated tool coordinate system. The global tool coordinate system will be used if it is not set.
        """
        string = "PositiveKin({:f},{:f},{:f},{:f},{:f},{:f}".format(
            J1, J2, J3, J4, J5, J6)
        params = []
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        for ii in params:
            string = string + ','+ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def InverseKin(self, X, Y, Z, Rx, Ry, Rz, user=-1, tool=-1, useJointNear=-1, JointNear=''):
        """
        描述
        進行逆解運算：給定機械臂末端在給定的笛卡爾座標系中的座標值，計算機械臂各關節角度。
        由於笛卡爾座標僅定義了TCP的空間座標與傾斜角，所以機械臂可以透過多種不同的姿態到達同一
        個位姿，意味著一個位姿變數可以對應多個關節變數。為得出唯一的解，系統需要一個指定的關節
        座標，選擇最接近該關節座標的解作為逆解結果。
        必選參數
        參數名 類型 說明
        X double X軸位置，單位：mm
        Y double Y軸位置，單位：mm
        Z double Z軸位置，單位：mm
        Rx double Rx軸位置，單位：度
        Ry double Ry軸位置，單位：度
        Rz double Rz軸位置，單位：度
        可選參數
        參數名 類型 說明
        User string  格式為"user=index"，index為已標定的使用者座標系索引。不指定時使用全域使用者座標系。
        Tool string  格式為"tool=index"，index為已標定的工具座標系索引。不指定時使用全域工具座標系。
        useJointNear string  用於設定JointNear參數是否有效。
            "useJointNear=0"或不攜帶表示JointNear無效，系統根據機械臂當前關節角度就近選解。
            "useJointNear=1"表示根據JointNear就近選解。
        jointNear string 格式為"jointNear={j1,j2,j3,j4,j5,j6}"，用於就近選解的關節座標。
        Description
        Inverse solution. Calculate the joint angles of the robot, based on the given coordinates in the specified Cartesian coordinate system.
        As Cartesian coordinates only define the spatial coordinates and tilt angle of the TCP, the robot arm can reach the same posture through different gestures, which means that one posture variable can correspond to multiple joint variables.
        To get a unique solution, the system requires a specified joint coordinate, and the solution closest to this joint coordinate is selected as the inverse solution。
        Required parameter:
        Parameter name     Type     Description
        X     double     X-axis position, unit: mm
        Y     double     Y-axis position, unit: mm
        Z     double     Z-axis position, unit: mm
        Rx     double     Rx-axis position, unit: °
        Ry     double     Ry-axis position, unit: °
        Rz     double     Rz-axis position, unit: °
        Optional parameter:
        Parameter name     Type     Description
        User      string     Format: "user=index", index: index of the calibrated user coordinate system. The global user coordinate system will be used if it is not set.
        Tool     string     Format: "tool=index", index: index of the calibrated tool coordinate system. The global tool coordinate system will be used if it is not set.
        useJointNear     string     used to set whether JointNear is effective.
            "useJointNear=0" or null: JointNear data is ineffective. The algorithm selects the joint angles according to the current angle.
            "useJointNear=1": the algorithm selects the joint angles according to JointNear data.
        jointNear     string     Format: "jointNear={j1,j2,j3,j4,j5,j6}", joint coordinates for selecting joint angles.
        """
        string = "InverseKin({:f},{:f},{:f},{:f},{:f},{:f}".format(
            X, Y, Z, Rx, Ry, Rz)
        params = []
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        if useJointNear != -1:
            params.append('useJointNear={:d}'.format(useJointNear))
        if JointNear != '':
            params.append('JointNear={:s}'.format(JointNear))
        for ii in params:
            string = string + ','+ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def GetAngle(self):
        """
        獲取機械臂當前位姿的關節座標。
        Get the joint coordinates of current posture.
        """
        string = "GetAngle()"
        return self.sendRecvMsg(string)

    def GetPose(self, user=-1, tool=-1):
        """
        獲取機械臂當前位姿在指定的座標系下的笛卡爾座標。
        可選參數
        參數名 類型 說明
        User string 格式為"user=index"，index為已標定的使用者座標系索引。
        Tool string 格式為"tool=index"，index為已標定的工具座標系索引。
        必須同時傳或同時不傳，不傳時預設為全域使用者和工具座標系。
        Get the Cartesian coordinates of the current posture under the specific coordinate system.
        Optional parameter:
        Parameter name     Type     Description
        User      string     Format: "user=index", index: index of the calibrated user coordinate system.
        Tool     string     Format: "tool=index", index: index of the calibrated tool coordinate system.
        They need to be set or not set at the same time. They are global user coordinate system and global tool coordinate system if not set.
        """
        string = "GetPose("
        params = []
        state = True
        if user != -1:
            params.append('user={:d}'.format(user))
            state = not state
        if tool != -1:
            params.append('tool={:d}'.format(tool))
            state = not state
        if not state:
            return 'need to be set or not set at the same time. They are global user coordinate system and global tool coordinate system if not set' # 必須同時傳或同時不傳座標系，不傳時預設為全域使用者和工具座標系

        for i, param in enumerate(params):
            if i == len(params)-1:
                string = string + param
            else:
                string = string + param+","

        string = string + ')'
        return self.sendRecvMsg(string)

    def GetErrorID(self):
        """
        獲取機械臂當前的錯誤 ID。
        Get the current error ID of the robot arm.
        """
        string = "GetErrorID()"
        return self.sendRecvMsg(string)

     #################################################################

    def DO(self, index, status, time=-1):
        """
        設定數位輸出埠狀態（佇列指令）。
        必選參數
        參數名 類型 說明
        index int DO端子的編號
        status int DO端子的狀態，1：ON；0：OFF
        可選參數
        參數名 類型  說明
        time int 持續輸出時間，取值範圍：[25, 60000]。單位：ms
        如果設定了該參數，系統會在指定時間後對DO自動取反。取反為非同步動作，
        不會阻塞指令佇列，系統執行了DO輸出後就會執行下一條指令。
        Set the status of digital output port (queue command).
        Required parameter:
        Parameter name     Type     Description
        index     int     DO index
        status     int     DO index, 1: ON, 0: OFF
        Optional parameter:
        Parameter name     Type     Description
        time     int     continuous output time, range: [25,60000]. Unit: ms.
        If this parameter is set, the system will automatically invert the DO after the specified time.
        The inversion is an asynchronous action, which will not block the command queue. After the DO output is executed, the system will execute the next command.
        """
        string = "DO({:d},{:d}".format(index, status)
        params = []
        if time != -1:
            params.append('{:d}'.format(time))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def DOInstant(self, index, status):
        """
        設定數位輸出埠狀態（立即指令）。
        必選參數
        參數名 類型 說明
        index int DO端子的編號
        status int DO端子的狀態，1：ON；0：OFF
        Set the status of digital output port (immediate command).
        Required parameter:
        Parameter name     Type     Description
        index     int     DO index
        status     int     DO index, 1: ON, 0: OFF
        """
        string = "DOInstant({:d},{:d})".format(index, status)
        return self.sendRecvMsg(string)

    def GetDO(self, index):
        """
        獲取數位輸出埠狀態。
        必選參數
        參數名 類型 說明
        index int DO端子的編號
        Get the status of digital output port.
        Required parameter:
        Parameter name     Type     Description
        index     int     DO index
        """
        string = "GetDO({:d})".format(index)
        return self.sendRecvMsg(string)

    def DOGroup(self, *index_value):
        """
        設定多個數位輸出埠狀態（佇列指令）。
        必選參數
        參數名 類型 說明
        index1 int 第一個DO端子的編號
        value1 int 第一個DO端子的狀態，1：ON；0：OFF
        ... ... ...
        indexN int 第N個DO端子的編號
        valueN int 第N個DO端子的狀態，1：ON；0：OFF
        回傳
        ErrorID,{ResultID},DOGroup(index1,value1,index2,value2,...,indexN,valueN);
        ResultID為演算法佇列ID，可用於判斷指令執行順序。
        範例
        DOGroup(4,1,6,0,2,1,7,0)
        設定DO_4為ON，DO_6為OFF，DO_2為ON，DO_7為OFF。
        Set the status of multiple digital output ports (queue command).
        Required parameter:
        Parameter name     Type     Description
        index1     int     index of the first DO
        value1      int     status of the first DO, 1: ON, 0: OFF
        ... ... ...
        indexN     int     index of the last DO
        valueN      int     status of the last DO, 1: ON, 0: OFF
        Return
        ErrorID,{ResultID},DOGroup(index1,value1,index2,value2,...,indexN,valueN);
        ResultID is algorithm queue ID, used to judge the order in which commands are executed.
        Example
        DOGroup(4,1,6,0,2,1,7,0)
        Set DO_4 to ON, DO_6 to OFF, DO_2 to ON, DO_7 to OFF.
        """
        string = "DOGroup({:d}".format(index_value[0])
        for ii in index_value[1:]:
            string = string + ',' + str(ii)
        string = string + ')'

        return self.sendRecvMsg(string)

    def GetDOGroup(self, *index_value):
        """
        獲取多個數位輸出埠狀態。
        必選參數
        參數名 類型 說明
        index int 第一個DO端子的編號
        ... ... ...
        indexN int 第N個DO端子的編號
        回傳
        ErrorID,{value1,value2,...,valueN},GetDOGroup(index1,index2,...,indexN);
        {value1,value2,...,valueN}分別表示DO_1到DO_N的狀態，0為OFF，1為ON
        範例
        GetDOGroup(1,2)
        獲取DO_1和DO_2的狀態。
        Get the status of multiple digital output ports.
        Required parameter:
        Parameter name     Type     Description
        index     int     index of the first DO
        ... ... ...
        indexN     int     index of the last DO
        Return
        ErrorID,{value1,value2,...,valueN},GetDOGroup(index1,index2,...,indexN);
        {value1,value2,...,valueN}: status of DO_1 – DO_N. 0: OFF, 1: ON.
        Example
        GetDOGroup(1,2)
        Get the status of DO_1 and DO_2.
        """
        string = "GetDOGroup({:d}".format(index_value[0])
        for ii in index_value[1:]:
            string = string + ',' + str(ii)
        string = string + ')'
        return self.sendRecvMsg(string)

    def ToolDO(self, index, status):
        """
        設定末端數位輸出埠狀態（佇列指令）。
        必選參數
        參數名 類型 說明
        index int 末端DO端子的編號
        status int 末端DO端子的狀態，1：ON；0：OFF
        Set the status of tool digital output port (queue command).
        Required parameter:
        Parameter name     Type     Description
        index     int     index of the tool DO
        status     int     status of the tool DO, 1: ON, 0: OFF
        """
        string = "ToolDO({:d},{:d})".format(index, status)
        return self.sendRecvMsg(string)

    def ToolDOInstant(self, index, status):
        """
        設定末端數位輸出埠狀態（立即指令）
        必選參數
        參數名 類型 說明
        index int 末端DO端子的編號
        status int 末端DO端子的狀態，1：ON；0：OFF
        Set the status of tool digital output port (immediate command)
        Required parameter:
        Parameter name     Type     Description
        index     int     index of the tool DO
        status     int     status of the tool DO, 1: ON, 0: OFF
        """
        string = "ToolDOInstant({:d},{:d})".format(index, status)
        return self.sendRecvMsg(string)

    def GetToolDO(self, index):
        """
        獲取末端數位輸出埠狀態
        必選參數
        參數名 類型 說明
        index int 末端DO端子的編號
        Get the status of tool digital output port
        Required parameter:
        Parameter name     Type     Description
        index     int     index of the tool DO
        """
        string = "GetToolDO({:d})".format(index)
        return self.sendRecvMsg(string)

    def AO(self, index, value):
        """
        設定類比輸出埠的值（佇列指令）。
        必選參數
        參數名 類型 說明
        index int AO端子的編號
        value double AO端子的輸出值，電壓取值範圍：[0,10]，單位：V；電流取值範圍：[4,20]，單位：mA
        Set the value of analog output port (queue command).
        Required parameter:
        Parameter name     Type     Description
        index     int     AO index
        value     double     AO output, voltage range: [0,10], unit: V; current range: [4,20], unit: mA
        """
        string = "AO({:d},{:f})".format(index, value)
        return self.sendRecvMsg(string)

    def AOInstant(self, index, value):
        """
        設定類比輸出埠的值（立即指令）。
        必選參數
        參數名 類型 說明
        index int AO端子的編號
        value double AO端子的輸出值，電壓取值範圍：[0,10]，單位：V；電流取值範圍：
        [4,20]，單位：mA
        Set the value of analog output port (immediate command).
        Required parameter:
        Parameter name     Type     Description
        index     int     AO index
        value     double     AO output, voltage range: [0,10], unit: V; current range:
        [4,20], unit: mA
        """
        string = "AOInstant({:d},{:f})".format(index, value)
        return self.sendRecvMsg(string)

    def GetAO(self, index):
        """
        獲取類比輸出埠的值。
        必選參數
        參數名 類型 說明
        index int AO端子的編號
        Get the value of analog output port.
        Required parameter:
        Parameter name     Type     Description
        index     int     AO index
        """
        string = "GetAO({:d})".format(index)
        return self.sendRecvMsg(string)

    def DI(self, index):
        """
        獲取DI埠的狀態。
        必選參數
        參數名 類型 說明
        index int DI端子的編號
        Get status of DI port.
        Required parameter:
        Parameter name     Type     Description
        index     int     DI index
        """
        string = "DI({:d})".format(index)
        return self.sendRecvMsg(string)

    def DIGroup(self, *index_value):
        """
        獲取多個DI埠的狀態。
        必選參數
        參數名 類型 說明
        index1 int 第一個DI端子的編號
        ... ... ...
        indexN int 第N個DI端子的編號
        回傳
        ErrorID,{value1,value2,...,valueN},DIGroup(index1,index2,...,indexN);
        {value1,value2,...,valueN}分別表示DI_1到DI_N的狀態，0為OFF，1為ON
        範例
        DIGroup(4,6,2,7)
        獲取DI_4，DI_6，DI_2，DI_7的狀態。
        Get status of multiple DI ports.
        Required parameter:
        Parameter name     Type     Description
        index1     int     index of the first DI
        ... ... ...
        indexN     int     index of the last DI
        Return
        ErrorID,{value1,value2,...,valueN},DIGroup(index1,index2,...,indexN);
        {value1,value2,...,valueN}: status of DI_1 – DI_N. 0: OFF, 1: ON.
        Example
        DIGroup(4,6,2,7)
        Get the status of DI_4, DI_6, DI_2 and DI_7.
        """
        string = "DIGroup({:d}".format(index_value[0])
        for ii in index_value[1:]:
            string = string + ',' + str(ii)
        string = string + ')'
        return self.sendRecvMsg(string)

    def ToolDI(self, index):
        """
        獲取末端DI埠的狀態。
        必選參數
        參數名 類型 說明
        index int 末端DI端子的編號
        Get the status of tool digital input port.
        Required parameter:
        Parameter name     Type     Description
        index     int     index of the tool DI
        """
        string = "ToolDI({:d})".format(index)
        return self.sendRecvMsg(string)

    def AI(self, index):
        """
        獲取AI埠的值。
        必選參數
        參數名 類型 說明
        index int AI端子的編號
        Get the value of analog input port.
        Required parameter:
        Parameter name     Type     Description
        index     int     AI index
        """
        string = "AI({:d})".format(index)
        return self.sendRecvMsg(string)

    def ToolAI(self, index):
        """
        獲取末端AI埠的值。使用前需要透過SetToolMode將端子設定為類比輸入模式。
        必選參數
        參數名 類型 說明
        index int 末端AI端子的編號
        Get the value of tool analog input port. You need to set the port to analog-input mode through SetToolMode before use.
        Required parameter:
        Parameter name     Type     Description
        index     int     index of the tool AI
        """
        string = "ToolAI({:d})".format(index)
        return self.sendRecvMsg(string)

    def SetTool485(self, index, parity='', stopbit=-1, identify=-1):
        """
        描述:
        設定末端工具的RS485介面對應的資料格式。
        必選參數
        參數名 類型 說明
        baud int RS485介面的鮑率
        可選參數
        參數名 類型 說明
        parity string
        是否有奇偶校驗位。"O"表示奇校驗，"E"表示偶校驗，"N"表示無奇偶
        校驗位。預設值為"N"。
        stopbit int 停止位長度。取值範圍：1，2。預設值為1。
        identify int 當機械臂為多航插機型時，用於指定設定的航插。1：航插1；2：航插2
        回傳
        ErrorID,{},SetTool485(baud,parity,stopbit);
        範例：
        SetTool485(115200,"N",1)
        將末端工具的RS485介面對應的鮑率設定為115200Hz，無奇偶校驗位，停止位長度為1。
        Description:
        Set the data type corresponding to the RS485 interface of the end tool.
        Required parameter:
        Parameter name     Type     Description
        baud     int     baud rate of RS485 interface
        Optional parameter:
        Parameter name     Type     Description
        parity string     Whether there are parity bits. "O" means odd, "E" means even, and "N" means no parity bit. "N" by default.
        stopbit     int     stop bit length Range: 1, 2 1 by default.
        identify     int     If the robot is equipped with multiple aviation sockets, you need to specify them. 1: aviation 1; 2: aviation 2
        Return
        ErrorID,{},SetTool485(baud,parity,stopbit);
        Example
        SetTool485(115200,"N",1)
        Set the baud rate corresponding to the tool RS485 interface to 115200Hz, parity bit to N, and stop bit length to 1.
        """
        string = "SetTool485({:d}".format(index)
        params = []
        if parity != '':
            params.append(parity)
        if string != -1:
            params.append('{:d}'.format(stopbit))
            if identify != -1:
                params.append('{:d}'.format(identify))
        else:
            if identify != -1:
                params.append('1,{:d}'.format(identify))  # 選擇航插未設停止位，預設為1  no stop bit, 1 by default
        for ii in params:
            string = string + ','+ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def SetToolPower(self, status, identify=-1):
        """
        設定末端工具供電狀態，一般用於重啟末端電源，例如對末端夾爪重新上電初始化。如需連續呼叫
        該介面，建議至少間隔4ms以上。
        說明：
        Magician E6機器人不支援該指令，呼叫無效果。
        必選參數
        參數名 類型 說明
        status int 末端工具供電狀態，0：關閉電源；1：開啟電源
        可選參數
        參數名 類型 說明
        identify int 當機械臂為多航插機型時，用於指定設定的航插。1：航插1；2：航插2
        回傳
        ErrorID,{},SetToolPower(status);
        範例：
        SetToolPower(0)
        關閉末端電源。
        Set the power status of the end tool, generally used for restarting the end power, such as re-powering and re-initializing the gripper.
        If you need to call the interface continuously, it is recommended to keep an interval of at least 4 ms.
        NOTE:
        This command is not supported on Magician E6 robot, and there is no effect when calling it.
        Required parameter:
        Parameter name     Type     Description
        status    int     power status of end tool. 0: power off; 1: power on.
        Optional parameter:
        Parameter name     Type     Description
        identify     int     If the robot is equipped with multiple aviation sockets, you need to specify them. 1: aviation 1; 2: aviation 2
        Return
        ErrorID,{},SetToolPower(status);
        Example
        SetToolPower(0)
        Power off the tool.
        """
        string = "SetToolPower({:d}".format(status)
        params = []
        if identify != -1:
            params.append('{:d}'.format(identify))
        for ii in params:
            string = string + ','+ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def SetToolMode(self, mode, type, identify=-1):
        """
        描述:
        機械臂末端AI介面與485介面複用端子時，可透過此介面設定末端複用端子的模式。預設模式為
        485模式。
        說明：
        不支援末端模式切換的機械臂呼叫此介面無效果。
        必選參數
        參數名 類型 說明
        mode int 複用端子的模式，1：485模式，2：類比輸入模式
        type int  當mode為1時，該參數無效。當mode為2時，可設定類比輸入的模式。
                  個位表示AI1的模式，十位表示AI2的模式，十位為0時可僅輸入個位。
        模式：
        0：0~10V電壓輸入模式
        1：電流採集模式
        2：0~5V電壓輸入模式
        範例：
        0：AI1與AI2均為0~10V電壓輸入模式
        1：AI2是0~10V電壓輸入模式，AI1是電流採集模式
        11：AI2和AI1都是電流採集模式
        12：AI2是電流採集模式，AI1是0~5V電壓輸入模式
        20：AI2是0~5V電壓輸入模式，AI1是0~10V電壓輸入模式
        可選參數
        參數名 類型 說明
        identify int 當機械臂為多航插機型時，用於指定設定的航插。1：航插1；2：航插2
        回傳
        ErrorID,{},SetToolMode(mode,type);
        範例：
        SetToolMode(2,0)
        設定末端複用端子為類比輸入，兩路都是0~10V電壓輸入模式。
        Description:
        If the AI interface on the end of the robot arm is multiplexed with the 485 interface, you can set the mode of the end multiplex terminal via this interface.
        485 mode by default.
        NOTE:
        The robot arm without tool RS485 interface has no effect when calling this interface.
        Required parameter:
        Parameter name     Type     Description
        mode     int     mode of the multiplex terminal, 1: 485 mode, 2: AI mode
        type     int     When mode is 1, this parameter is invalid. When mode is 2, you can set the mode of AI.
                  The single digit indicates the mode of AI1, the tens digit indicates the mode of AI2. When the tens digit is 0, you can enter only the single digit.
        Mode:
        0: 0 – 10V voltage input mode
        1: Current collection mode
        2: 0 – 5V voltage input mode
        Example:
        0: AI1 and AI2 are 0 – 10V voltage input mode
        1: AI2 is 0 – 10V voltage input mode, AI1 is current collection mode
        11: AI2 and AI1 are current collection mode
        12: AI2 is current collection mode, AI1 is 0 – 5V voltage input mode
        20: AI2 is 0 – 5V voltage input mode, AI1 is 0 – 10V voltage input mode
        Optional parameter:
        Parameter name     Type     Description
        identify     int     If the robot is equipped with multiple aviation sockets, you need to specify them. 1: aviation 1; 2: aviation 2
        Return
        ErrorID,{},SetToolMode(mode,type);
        Example
        SetToolMode(2,0)
        Set the mode of the end multiplex terminal to AI, both are 0 – 10V voltage input mode.
        """
        string = "SetToolMode({:d},{:d}".format(mode, type)
        params = []
        if identify != -1:
            params.append('{:d}'.format(identify))
        for ii in params:
            string = string + ','+ii
        string = string + ')'
        return self.sendRecvMsg(string)

     ##################################################################

    def ModbusCreate(self, ip, port, slave_id, isRTU=-1):
        """
        建立Modbus主站，並和從站建立連線。最多支援同時連接5個設備。
        必選參數
        參數名 類型 說明
        ip string 從站IP位址
        port int 從站埠
        slave_id int 從站ID
        可選參數
        參數名 類型 說明
        isRTU int 如果不攜帶或為0，建立modbusTCP通訊；如果為1，建立modbusRTU通訊
        Create Modbus master, and establish connection with the slave. (support connecting to at most 5 devices).
        Required parameter:
        Parameter name     Type     Description
        ip     string     slave IP address
        port     int     slave port
        slave_id     int     slave ID
        Optional parameter:
        Parameter name     Type     Description
        isRTU     int     null or 0: establish ModbusTCP communication; 1: establish ModbusRTU communication
        """
        string = "ModbusCreate({:s},{:d},{:d}".format(ip, port, slave_id)
        params = []
        if isRTU != -1:
            params.append('{:d}'.format(isRTU))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def ModbusRTUCreate(self, slave_id, baud, parity='', data_bit=8, stop_bit=-1):
        """
        建立基於RS485介面的Modbus主站，並和從站建立連線。最多支援同時連接5個設備。
        必選參數
        參數名 類型 說明
        slave_id int 從站ID
        baud int RS485介面的鮑率。
        可選參數
        參數名 類型 說明
        parity string
        是否有奇偶校驗位。"O"表示奇校驗，"E"表示偶校驗，"N"表示無奇偶
        校驗位。預設值為"E"。
        data_bit int 資料位長度。取值範圍：8。預設值為8。
        stop_bit int 停止位長度。取值範圍：1，2。預設值為1。
        Create Modbus master station based on RS485, and establish connection with slave station (support connecting to at most 5 devices).
        Required parameter:
        Parameter name     Type     Description
        slave_id     int     slave ID
        baud     int     baud rate of RS485 interface.
        Optional parameter:
        Parameter name     Type     Description
        parity string     Whether there are parity bits. "O" means odd, "E" means even, and "N" means no parity bit. "E" by default.
        data_bit     int     data bit length Range: 8 (8 by default).
        stop_bit     int     stop bit length Range: 1, 2 (1 by default).
        """
        string = "ModbusRTUCreate({:d},{:d}".format(slave_id, baud)
        params = []
        if parity != '':
            params.append('{:s}'.format(parity))
        if data_bit != 8:
            params.append('{:d}'.format(data_bit))
        if stop_bit != -1:
            params.append('{:d}'.format(stop_bit))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def ModbusClose(self, index):
        """
        和Modbus從站斷開連線，釋放主站。
        必選參數
        參數名 類型 說明
        index int 建立主站時回傳的主站索引
        Disconnect with Modbus slave and release the master.
        Required parameter:
        Parameter name     Type     Description
        index     int     master index
        """
        string = "ModbusClose({:d})".format(index)
        return self.sendRecvMsg(string)

    def GetInBits(self, index, addr, count):
        """
        讀取Modbus從站觸點暫存器（離散輸入）位址的值。
        必選參數
        參數名 類型 說明
        index int 建立主站時回傳的主站索引
        addr int 觸點暫存器起始位址
        count int 連續讀取觸點暫存器的值的數量。取值範圍：[1, 16]
        Read the contact register (discrete input) value from the Modbus slave.
        Required parameter:
        Parameter name     Type     Description
        index     int     master index
        addr     int     starting address of the contact register
        count     int     number of contact registers Range: [1, 16].
        """
        string = "GetInBits({:d},{:d},{:d})".format(index, addr, count)
        return self.sendRecvMsg(string)

    def GetInRegs(self, index, addr, count, valType=''):
        """
        按照指定的資料類型，讀取Modbus從站輸入暫存器位址的值。
        必選參數
        參數名 類型 說明
        index int 建立主站時回傳的主站索引
        addr int 輸入暫存器起始位址
        count int 連續讀取輸入暫存器的值的數量。取值範圍：[1, 4]
        可選參數
        參數名 類型 說明
        valType string
        讀取的資料類型：
        U16：16位無符號整數（2個位元組，佔用1個暫存器）；
        U32：32位無符號整數（4個位元組，佔用2個暫存器）
        F32：32位單精度浮點數（4個位元組，佔用2個暫存器）
        F64：64位雙精度浮點數（8個位元組，佔用4個暫存器）
        預設為U16
        Read the input register value with the specified data type from the Modbus slave.
        Required parameter:
        Parameter name     Type     Description
        index     int     master index
        addr     int     starting address of the input register
        count     int     number of input registers Range: [1, 4].
        Optional parameter:
        Parameter name     Type     Description
        valType string
        Data type:
        U16: 16-bit unsigned integer (two bytes, occupy one register)
        U32: 32-bit unsigned integer (four bytes, occupy two register).
        F32: 32-bit single-precision floating-point number (four bytes, occupy two registers)
        F64: 64-bit double-precision floating-point number (eight bytes, occupy four registers)
        U16 by default.
        """
        string = "GetInRegs({:d},{:d},{:d}".format(index, addr, count)
        params = []
        if valType != '':
            params.append('{:s}'.format(valType))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def GetCoils(self, index, addr, count):
        """
        讀取Modbus從站線圈暫存器位址的值。
        必選參數
        參數名 類型 說明
        index int 建立主站時回傳的主站索引
        addr int 線圈暫存器起始位址
        count int 連續讀取線圈暫存器的值的數量。取值範圍：[1, 16]
        Read the coil register value from the Modbus slave.
        Required parameter:
        Parameter name     Type     Description
        index     int     master index
        addr     int     starting address of the coil register
        count     int     number of coil registers Range: [1, 16].
        """
        string = "GetCoils({:d},{:d},{:d})".format(index, addr, count)
        return self.sendRecvMsg(string)

    def SetCoils(self, index, addr, count, valTab):
        """
        描述
        將指定的值寫入線圈暫存器指定的位址。
        必選參數
        參數名 類型 說明
        index int 建立主站時回傳的主站索引
        addr int 線圈暫存器起始位址
        count int 連續寫入線圈暫存器的值的數量。取值範圍：[1, 16]
        valTab string 要寫入的值，數量與count相同
        回傳
        ErrorID,{},SetCoils(index,addr,count,valTab);
        範例
        SetCoils(0,1000,3,{1,0,1})
        從位址為1000的線圈暫存器開始連續寫入3個值，分別為1，0，1。
        Description
        Write the specified value to the specified address of coil register.
        Required parameter:
        Parameter name     Type     Description
        index     int     master index
        addr     int     starting address of the coil register
        count     int     number of values to be written to the coil register. Range: [1, 16].
        valTab     string     values to be written to the register (number of values equals to count)
        Return
        ErrorID,{},SetCoils(index,addr,count,valTab);
        Example
        SetCoils(0,1000,3,{1,0,1})
        Write three values (1 , 0, 1) to the coil register starting from address 1000.
        """
        string = "SetCoils({:d},{:d},{:d},{:s})".format(
            index, addr, count, valTab)
        return self.sendRecvMsg(string)

    def GetHoldRegs(self, index, addr, count, valType=''):
        """
        按照指定的資料類型，讀取Modbus從站保持暫存器位址的值。
        必選參數
        參數名 類型 說明
        index int 建立主站時回傳的主站索引
        addr int 保持暫存器起始位址
        count int 連續讀取保持暫存器的值的數量。取值範圍：[1, 4]
        可選參數
        參數名 類型 說明
        valType string
        讀取的資料類型：
        U16：16位無符號整數（2個位元組，佔用1個暫存器）；
        U32：32位無符號整數（4個位元組，佔用2個暫存器）
        F32：32位單精度浮點數（4個位元組，佔用2個暫存器）
        F64：64位雙精度浮點數（8個位元組，佔用4個暫存器）
        預設為U16
        Write the specified value according to the specified data type to the specified address of holding register.
        Required parameter:
        Parameter name     Type     Description
        index     int     master index
        addr     int     starting address of the holding register
        count     int     number of values to be written to the holding register. Range: [1, 4].
        Optional parameter:
        Parameter name     Type     Description
        valType string
        Data type:
        U16: 16-bit unsigned integer (two bytes, occupy one register)
        U32: 32-bit unsigned integer (four bytes, occupy two register)
        F32: 32-bit single-precision floating-point number (four bytes, occupy two registers)
        F64: 64-bit double-precision floating-point number (eight bytes, occupy four registers)
        U16 by default.
        """
        string = "GetHoldRegs({:d},{:d},{:d}".format(index, addr, count)
        params = []
        if valType != '':
            params.append('{:s}'.format(valType))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def SetHoldRegs(self, index, addr, count, valTab, valType=''):
        """
        將指定的值以指定的資料類型寫入Modbus從站保持暫存器指定的位址。
        必選參數
        參數名 類型 說明
        index int 建立主站時回傳的主站索引
        addr int 保持暫存器起始位址
        count int 連續寫入保持暫存器的值的數量。取值範圍：[1, 4]
        valTab string 要寫入的值，數量與count相同。
        可選參數
        參數名 類型 說明
        valType string
        寫入的資料類型：
        U16：16位無符號整數（2個位元組，佔用1個暫存器）；
        U32：32位無符號整數（4個位元組，佔用2個暫存器）
        F32：32位單精度浮點數（4個位元組，佔用2個暫存器）
        F64：64位雙精度浮點數（8個位元組，佔用4個暫存器）
        預設為U16
        Write the specified value with specified data type to the specified address of holding register.
        Required parameter:
        Parameter name     Type     Description
        index     int     master index
        addr     int     starting address of the holding register
        count     int     number of values to be written to the holding register. Range: [1, 4].
        valTab     string     values to be written to the register (number of values equals to count).
        Optional parameter:
        Parameter name     Type     Description
        valType string
        Data type:
        U16: 16-bit unsigned integer (two bytes, occupy one register)
        U32: 32-bit unsigned integer (four bytes, occupy two register)
        F32: 32-bit single-precision floating-point number (four bytes, occupy two registers)
        F64: 64-bit double-precision floating-point number (eight bytes, occupy four registers)
        U16 by default.
        """
        string = "SetHoldRegs({:d},{:d},{:d},{:s}".format(
            index, addr, count, valTab)
        params = []
        if valType != '':
            params.append('{:s}'.format(valType))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)
    ########################################################################

    def GetInputBool(self, address):
        """
        獲取輸入暫存器指定位址的bool類型的數值。
        必選參數
        參數名 類型 說明
        address int 暫存器位址，取值範圍[0-63]
        Get the value in bool type from the specified address of input register.
        Required parameter:
        Parameter name     Type     Description
        address     int     register address, range: [0-63]
        """
        string = "GetInputBool({:d})".format(address)
        return self.sendRecvMsg(string)

    def GetInputInt(self, address):
        """
        獲取輸入暫存器指定位址的int類型的數值。
        必選參數
        參數名 類型 說明
        address int 暫存器位址，取值範圍[0-23]
        Get the value in int type from the specified address of input register.
        Required parameter:
        Parameter name     Type     Description
        address     int     register address, range: [0-23]
        """
        string = "GetInputInt({:d})".format(address)
        return self.sendRecvMsg(string)

    def GetInputFloat(self, address):
        """
        獲取輸入暫存器指定位址的float類型的數值。
        必選參數
        參數名 類型 說明
        address int 暫存器位址，取值範圍[0-23]
        Get the value in float type from the specified address of input register.
        Required parameter:
        Parameter name     Type     Description
        address     int     register address, range: [0-23]
        """
        string = "GetInputFloat({:d})".format(address)
        return self.sendRecvMsg(string)

    def GetOutputBool(self, address):
        """
        獲取輸出暫存器指定位址的bool類型的數值。
        必選參數
        參數名 類型 說明
        address int 暫存器位址，取值範圍[0-63]
        Get the value in bool type from the specified address of output register.
        Required parameter:
        Parameter name     Type     Description
        address     int     register address, range: [0-63]
        """
        string = "GetOutputBool({:d})".format(address)
        return self.sendRecvMsg(string)

    def GetOutputInt(self, address):
        """
        獲取輸出暫存器指定位址的int類型的數值。
        必選參數
        參數名 類型 說明
        address int 暫存器位址，取值範圍[0-23]
        Get the value in int type from the specified address of output register.
        Required parameter:
        Parameter name     Type     Description
        address     int     register address, range: [0-23]
        """
        string = "GetOutputInt({:d})".format(address)
        return self.sendRecvMsg(string)

    def GetOutputFloat(self, address):
        """
        獲取輸出暫存器指定位址的float類型的數值。
        必選參數
        參數名 類型 說明
        address int 暫存器位址，取值範圍[0-23]
        Get the value in float type from the specified address of output register.
        Required parameter:
        Parameter name     Type     Description
        address     int     register address, range: [0-23]
        """
        string = "GetInputFloat({:d})".format(address)
        return self.sendRecvMsg(string)

    def SetOutputBool(self, address, value):
        """
        設定輸出暫存器指定位址的bool類型的數值。
        必選參數
        參數名 類型 說明
        address int 暫存器位址，取值範圍[0-63]
        value int 要設定的值，支援0或1
        Set the value in bool type at the specified address of output register.
        Required parameter:
        Parameter name     Type     Description
        address     int     register address, range: [0-63]
        value     int     value to be set (0 or 1)
        """
        string = "GetInputFloat({:d},{:d})".format(address, value)
        return self.sendRecvMsg(string)

    def SetOutputInt(self, address, value):
        """
        設定輸出暫存器指定位址的int類型的數值。
        必選參數
        參數名 類型 說明
        address int 暫存器位址，取值範圍[0-23]
        value int 要設定的值，支援整數
        Set the value in int type at the specified address of output register.
        Required parameter:
        Parameter name     Type     Description
        address     int     register address, range: [0-23]
        value     int     value to be set (integer)
        """
        string = "SetOutputInt({:d},{:d})".format(address, value)
        return self.sendRecvMsg(string)

    def SetOutputFloat(self, address, value):
        """
        設定輸出暫存器指定位址的float類型的數值。
        必選參數
        參數名 類型 說明
        address int 暫存器位址，取值範圍[0-23]
        value float 要設定的值，支援浮點數
        Set the value in float type at the specified address of output register.
        Required parameter:
        Parameter name     Type     Description
        address     int     register address, range: [0-23]
        value     float     value to be set (float)
        """
        string = "SetOutputFloat({:d},{:d})".format(address, value)
        return self.sendRecvMsg(string)

    #######################################################################

    def MovJ(self, a1, b1, c1, d1, e1, f1, coordinateMode, user=-1, tool=-1, a=-1, v=-1, cp=-1):
        """
        描述
        從當前位置以關節運動方式運動至目標點。
        必選參數
        參數名 類型 說明
        P string 目標點，支援關節變數或位姿變數
        coordinateMode int  目標點的座標值模式    0為pose方式  1為joint
        可選參數
        參數名 類型 說明
        user int 使用者座標系
        tool int 工具座標系
        a int 執行該條指令時的機械臂運動加速度比例。取值範圍：(0,100]
        v int 執行該條指令時的機械臂運動速度比例。取值範圍：(0,100]
        cp int 平滑過渡比例。取值範圍：[0,100]
        Description
        Move from the current position to the target position through joint motion.
        Required parameter:
        Parameter name     Type     Description
        P     string     Target point (joint variables or posture variables)
        coordinateMode     int      Coordinate mode of the target point, 0: pose, 1: joint
        Optional parameter:
        Parameter name     Type     Description
        user     int     user coordinate system
        tool     int     tool coordinate system
        a     int     acceleration rate of the robot arm when executing this command. Range: (0,100].
        v     int     velocity rate of the robot arm when executing this command. Range: (0,100].
        cp     int     continuous path rate. Range: [0,100].
        """
        string = ""
        if coordinateMode == 0:
            string = "MovJ(pose={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(
                a1, b1, c1, d1, e1, f1)
        elif coordinateMode == 1:
            string = "MovJ(joint={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(
                a1, b1, c1, d1, e1, f1)
        else:
            print("coordinateMode param is wrong")
            return ""
        params = []
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        if a != -1:
            params.append('a={:d}'.format(a))
        if v != -1:
            params.append('v={:d}'.format(v))
        if cp != -1:
            params.append('cp={:d}'.format(cp))
        for ii in params:
            string = string + ','+ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def MovL(self, a1, b1, c1, d1, e1, f1, coordinateMode, user=-1, tool=-1, a=-1, v=-1, speed=-1, cp=-1, r=-1):
        """
        描述
        從當前位置以直線運動方式運動至目標點。
        必選參數
        參數名 類型 說明
        P string 目標點，支援關節變數或位姿變數
        coordinateMode int  目標點的座標值模式    0為pose方式  1為joint
        可選參數
        參數名 類型  說明
        user int 使用者座標系
        tool int 工具座標系
        a    int 執行該條指令時的機械臂運動加速度比例。取值範圍：(0,100]
        v    int 執行該條指令時的機械臂運動速度比例，與speed互斥。取值範圍：(0,100]
        speed int 執行該條指令時的機械臂運動目標速度，與v互斥，若同時存在以speed為
        準。取值範圍：[1, 最大運動速度]，單位：mm/s
        cp  int 平滑過渡比例，與r互斥。取值範圍：[0,100]
        r   int 平滑過渡半徑，與cp互斥，若同時存在以r為準。單位：mm
        Description
        Move from the current position to the target position in a linear mode.
        Required parameter:
        Parameter name     Type     Description
        P     string     Target point (joint variables or posture variables)
        coordinateMode     int      Coordinate mode of the target point, 0: pose, 1: joint
        Optional parameter:
        Parameter name     Type     Description
        user     int     user coordinate system
        tool     int     tool coordinate system
        a     int     acceleration rate of the robot arm when executing this command. Range: (0,100].
        v     int     velocity rate of the robot arm when executing this command, incompatible with "speed". Range: (0,100].
        speed     int     target speed of the robot arm when executing this command, incompatible with "v". If both "speed" and "v" exist, speed takes precedence. Range: [0, maximum motion speed], unit: mm/s.
        cp     int     continuous path rate, incompatible with "r". Range: [0,100].
        r     int     continuous path radius, incompatible with "cp". If both "r" and "cp" exist, r takes precedence. Unit: mm.
        """
        string = ""
        if coordinateMode == 0:
            string = "MovL(pose={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(
                a1, b1, c1, d1, e1, f1)
        elif coordinateMode == 1:
            string = "MovL(joint={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(
                a1, b1, c1, d1, e1, f1)
        else:
            print("coordinateMode  param  is wrong")
            return ""
        params = []
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        if a != -1:
            params.append('a={:d}'.format(a))
        if v != -1 and speed != -1:
            params.append('speed={:d}'.format(speed))
        elif speed != -1:
            params.append('speed={:d}'.format(speed))
        elif v != -1:
            params.append('v={:d}'.format(v))
        if cp != -1 and r != -1:
            params.append('r={:d}'.format(r))
        elif r != -1:
            params.append('r={:d}'.format(r))
        elif cp != -1:
            params.append('cp={:d}'.format(cp))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def ServoJ(self, J1, J2, J3, J4, J5, J6, t=-1.0,aheadtime=-1.0, gain=-1.0):
        """
        參數名 類型 含義
        參數範圍
        J1 double 點J1 軸位置，單位：度 是
        J2 double 點J2 軸位置，單位：度 是
        J3 double 點J3 軸位置，單位：度 是
        J4 double 點J4 軸位置，單位：度 是
        J5 double 點J5 軸位置，單位：度 是
        J6 double 點J6 軸位置，單位：度 是
        t float 該點位的運行時間，預設0.1,單位：s 否 [0.004,3600.0]
        aheadtime float 作用類似於PID的D項，預設50，標量，無單位 否 [20.0,100.0]
        gain float 目標位置的比例放大器，作用類似於PID的P項，預設500，標量，無單位 否 [200.0,1000.0]
        Joint string Target point joint variables
        t float Optional parameter.Running time of the point, unit: s, value range: [0.02,3600.0], default value:0.1
        aheadtime float Optional parameter.Advanced time, similar to the D in PID control. Scalar, no unit, valuerange: [20.0,100.0], default value: 50.
        gain float Optional parameter.Proportional gain of the target position, similar to the P in PID control.Scalar, no unit, value range: [200.0,1000.0], default value: 500.
        """
        string = ""
        string = "ServoJ({:f},{:f},{:f},{:f},{:f},{:f}".format(J1, J2, J3, J4, J5, J6)
        params = []
        if t != -1:
            params.append('t={:f}'.format(t))
        if aheadtime != -1:
            params.append('aheadtime={:f}'.format(aheadtime))
        if gain != -1:
            params.append('gain={:f}'.format(gain))
        for ii in params:
            string = string + ','+ii
        string = string + ')'
        return self.sendRecvMsg(string)
    def ServoP(self, X, Y, Z, RX, RY, RZ, t=-1.0,aheadtime=-1.0, gain=-1.0):
        """
        參數名 類型 含義 是否必填 參數範圍
        X double X 軸位置，單位：毫米 是
        Y double Y 軸位置，單位：毫米 是
        Z double Z 軸位置，單位：毫米 是
        Rx double Rx 軸位置，單位：度 是
        Ry double Ry 軸位置，單位：度 是
        Rz double Rz 軸位置，單位：度 是
        t float 該點位的運行時間，預設0.1,單位：s 否 [0.004,3600.0]
        aheadtime float 作用類似於PID的D項，預設50，標量，無單位 否 [20.0,100.0]
        gain float 目標位置的比例放大器，作用類似於PID的P項，預設500，標量，無單位 否 [200.0,1000.0]
        Pose string  Target point posture variables. The reference coordinate system is the global user and tool coordinate system, see the User and Tool command descriptions in Settings command (the default values are both 0
        t float Optional parameter.Running time of the point, unit: s, value range: [0.02,3600.0], default value:0.1
        aheadtime float Optional parameter.Advanced time, similar to the D in PID control. Scalar, no unit, valuerange: [20.0,100.0], default value: 50.
        gain float Optional parameter.Proportional gain of the target position, similar to the P in PID control.Scalar, no unit, value range: [200.0,1000.0], default value: 500.
        """
        string = ""
        string = "ServoP({:f},{:f},{:f},{:f},{:f},{:f}".format(X, Y, Z, RX, RY, RZ)
        params = []
        if t != -1:
            params.append('t={:f}'.format(t))
        if aheadtime != -1:
            params.append('aheadtime={:f}'.format(aheadtime))
        if gain != -1:
            params.append('gain={:f}'.format(gain))
        for ii in params:
            string = string + ','+ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def MovLIO(self, a1, b1, c1, d1, e1, f1, coordinateMode, Mode, Distance, Index, Status, user=-1, tool=-1, a=-1, v=-1, speed=-1, cp=-1, r=-1):
        """
        描述
        從當前位置以直線運動方式運動至目標點，運動時並行設定數位輸出埠狀態。
        必選參數
        參數名 類型 說明
        P string 目標點，支援關節變數或位姿變數
        coordinateMode int  目標點的座標值模式    0為pose方式  1為joint
        {Mode,Distance,Index,Status}為並行數位輸出參數，用於設定當機械臂運動到指定距離或百分比
        時，觸發指定DO。可設定多組，參數具體含義如下：
        參數名 類型 說明
        Mode int 觸發模式。0表示距離百分比，1表示距離數值
        Distance int 指定距離。
        Distance為正數時，表示離起點的距離；
        Distance為負數時，表示離目標點的距離；
        Mode為0時，Distance表示和總距離的百分比；取值範圍：(0,100]；
        Mode為1時，Distance表示距離的值。單位：mm
        Index int DO端子的編號
        Status int 要設定的DO狀態，0表示無訊號，1表示有訊號
        可選參數
        參數名  類型  說明
        user int 使用者座標系
        tool int 工具座標系
        a int 執行該條指令時的機械臂運動加速度比例。取值範圍：(0,100]
        v int 執行該條指令時的機械臂運動速度比例，與speed互斥。取值範圍：(0,100]
        speed int 執行該條指令時的機械臂運動目標速度，與v互斥，若同時存在以speed為
        準。取值範圍：[1, 最大運動速度]，單位：mm/s
        cp int 平滑過渡比例，與r互斥。取值範圍：[0,100]
        r int 平滑過渡半徑，與cp互斥，若同時存在以r為準。單位：mm
        Description
        Move from the current position to the target position in a linear mode, and set the status of digital output port when the robot is moving.
        Required parameter:
        Parameter name     Type     Description
        P     string     Target point (joint variables or posture variables)
        coordinateMode     int      Coordinate mode of the target point, 0: pose, 1: joint
        {Mode,Distance,Index,Status}: digital output parameters, used to set the specified DO to be triggered when the robot arm moves to a specified distance or percentage. Multiple groups of parameters can be set.
        Parameter name     Type     Description
        Mode     int     Trigger mode. 0: distance percentage, 1: distance value.
        Distance     int     Specified distance.
        If Distance is positive, it refers to the distance away from the starting point;
        If Distance is negative, it refers to the distance away from the target point;
        If Mode is 0, Distance refers to the percentage of total distance. Range: (0,100];
        If Mode is 1, Distance refers to the distance value. Unit: mm.
        Index     int     DO index
        Status     int     DO status. 0: no signal, 1: have signal.
        Optional parameter:
        Parameter name     Type     Description
        user     int     user coordinate system
        tool     int     tool coordinate system
        a     int     acceleration rate of the robot arm when executing this command. Range: (0,100].
        v     int     velocity rate of the robot arm when executing this command, incompatible with "speed". Range: (0,100].
        speed     int     target speed of the robot arm when executing this command, incompatible with "v". If both "speed" and "v" exist, speed takes precedence. Range: [0, maximum motion speed], unit: mm/s.
        cp     int     continuous path rate, incompatible with "r". Range: [0,100].
        r     int     continuous path radius, incompatible with "cp". If both "r" and "cp" exist, r takes precedence. Unit: mm.
        """
        string = ""
        if coordinateMode == 0:
            string = "MovLIO(pose={{{:f},{:f},{:f},{:f},{:f},{:f}}},{{{:d},{:d},{:d},{:d}}}".format(
                a1, b1, c1, d1, e1, f1, Mode, Distance, Index, Status)
        elif coordinateMode == 1:
            string = "MovLIO(joint={{{:f},{:f},{:f},{:f},{:f},{:f}}},{{{:d},{:d},{:d},{:d}}}".format(
                a1, b1, c1, d1, e1, f1, Mode, Distance, Index, Status)
        else:
            print("coordinateMode  param  is wrong")
            return ""
        params = []
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        if a != -1:
            params.append('a={:d}'.format(a))
        if v != -1 and speed != -1:
            params.append('speed={:d}'.format(speed))
        elif speed != -1:
            params.append('speed={:d}'.format(speed))
        elif v != -1:
            params.append('v={:d}'.format(v))
        if cp != -1 and r != -1:
            params.append('r={:d}'.format(r))
        elif r != -1:
            params.append('r={:d}'.format(r))
        elif cp != -1:
            params.append('cp={:d}'.format(cp))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def MovJIO(self,  a1, b1, c1, d1, e1, f1, coordinateMode, Mode, Distance, Index, Status, user=-1, tool=-1, a=-1, v=-1, cp=-1,):
        """
        描述
        從當前位置以關節運動方式運動至目標點，運動時並行設定數位輸出埠狀態。
        必選參數
        參數名 類型 說明
        P string 目標點，支援關節變數或位姿變數
        coordinateMode int  目標點的座標值模式    0為pose方式  1為joint
        {Mode,Distance,Index,Status}為並行數位輸出參數，用於設定當機械臂運動到指定距離或百分比
        時，觸發指定DO。可設定多組，參數具體含義如下：
        參數名   類型  說明
        Mode int 觸發模式。0表示距離百分比，1表示距離數值。系統會將各關節角合成
        一個角度向量，並計算終點和起點的角度差作為運動的總距離。
        Distance int 指定距離。
        Distance為正數時，表示離起點的距離；
        Distance為負數時，表示離目標點的距離；
        Mode為0時，Distance表示和總距離的百分比；取值範圍：(0,100]；
        Mode為1時，Distance表示距離的角度。單位：°
        Index int DO端子的編號
        Status int 要設定的DO狀態，0表示無訊號，1表示有訊號
        可選參數
        參數名 類型 說明
        user int 使用者座標系
        tool int 工具座標系
        a int 執行該條指令時的機械臂運動加速度比例。取值範圍：(0,100]
        v int 執行該條指令時的機械臂運動速度比例。取值範圍：(0,100]
        cp int 平滑過渡比例。取值範圍：[0,100]
        Description
        Move from the current position to the target position through joint motion, and set the status of digital output port when the robot is moving.
        Required parameter:
        Parameter name     Type     Description
        P     string     Target point (joint variables or posture variables)
        coordinateMode     int      Coordinate mode of the target point, 0: pose, 1: joint
        {Mode,Distance,Index,Status}: digital output parameters, used to set the specified DO to be triggered when the robot arm moves to a specified distance or percentage. Multiple groups of parameters can be set.
        Parameter name     Type     Description
        Mode     int     Trigger mode. 0: distance percentage, 1: distance value.
        The system will synthesise the joint angles into an angular vector and calculate the angular difference between the end point and the start point as the total distance of the motion.
        Distance     int     Specified distance.
        If Distance is positive, it refers to the distance away from the starting point;
        If Distance is negative, it refers to the distance away from the target point;
        If Mode is 0, Distance refers to the percentage of total distance. Range: (0,100];
        If Mode is 1, Distance refers to the distance value. Unit: °.
        Index     int     DO index
        Status     int     DO status. 0: no signal, 1: have signal.
        Optional parameter:
        Parameter name     Type     Description
        user     int     user coordinate system
        tool     int     tool coordinate system
        a     int     acceleration rate of the robot arm when executing this command. Range: (0,100].
        v     int     velocity rate of the robot arm when executing this command. Range: (0,100].
        cp     int     continuous path rate. Range: [0,100].
        """
        string = ""
        if coordinateMode == 0:
            string = "MovJIO(pose={{{:f},{:f},{:f},{:f},{:f},{:f}}},{{{:d},{:d},{:d},{:d}}}".format(
                a1, b1, c1, d1, e1, f1, Mode, Distance, Index, Status)
        elif coordinateMode == 1:
            string = "MovJIO(joint={{{:f},{:f},{:f},{:f},{:f},{:f}}},{{{:d},{:d},{:d},{:d}}}".format(
                a1, b1, c1, d1, e1, f1, Mode, Distance, Index, Status)
        else:
            print("coordinateMode  param  is wrong")
            return ""
        params = []
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        if a != -1:
            params.append('a={:d}'.format(a))
        if v != -1:
            params.append('v={:d}'.format(v))
        if cp != -1:
            params.append('cp={:d}'.format(cp))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def Arc(self, a1, b1, c1, d1, e1, f1,  a2, b2, c2, d2, e2, f2, coordinateMode, user=-1, tool=-1, a=-1, v=-1, speed=-1, cp=-1, r=-1):
        """
        描述
        從當前位置以圓弧插補方式運動至目標點。
        需要透過當前位置、圓弧中間點、運動目標點三個點確定一個圓弧，因此當前位置不能在P1和P2
        確定的直線上。
        必選參數
        參數名 類型 說明
        P1 string 圓弧中間點，支援關節變數或位姿變數
        P2 string 運動目標點，支援關節變數或位姿變數
        coordinateMode int  目標點的座標值模式    0為pose方式  1為joint
        可選參數
        參數名  類型  說明
        user int 使用者座標系
        tool int 工具座標系
        a int  執行該條指令時的機械臂運動加速度比例。取值範圍：(0,100]
        v int 執行該條指令時的機械臂運動速度比例，與speed互斥。取值範圍：(0,100]
        speed int 執行該條指令時的機械臂運動目標速度，與v互斥，若同時存在以speed為
        準。取值範圍：[1, 最大運動速度]，單位：mm/s
        cp int 平滑過渡比例，與r互斥。取值範圍：[0,100]
        r int 平滑過渡半徑，與cp互斥，若同時存在以r為準。單位：mm
        Description
        Move from the current position to the target position in an arc interpolated mode.
        As the arc needs to be determined through the current position, through point and target point, the current position should not be in a straight line determined by P1 and P2.
        Required parameter:
        Parameter name     Type     Description
        P1     string     Through point (joint variables or posture variables)
        P2     string     Target point (joint variables or posture variables)
        coordinateMode     int      Coordinate mode of the target point, 0: pose, 1: joint
        Optional parameter:
        Parameter name     Type     Description
        user     int     user coordinate system
        tool     int     tool coordinate system
        a     int     acceleration rate of the robot arm when executing this command. Range: (0,100].
        v     int     velocity rate of the robot arm when executing this command, incompatible with "speed". Range: (0,100].
        speed     int     target speed of the robot arm when executing this command, incompatible with "v". If both "speed" and "v" exist, speed takes precedence. Range: [0, maximum motion speed], unit: mm/s.
        cp     int     continuous path rate, incompatible with "r". Range: [0,100].
        r     int     continuous path radius, incompatible with "cp". If both "r" and "cp" exist, r takes precedence. Unit: mm.
        """
        string = ""
        if coordinateMode == 0:
            string = "Arc(pose={{{:f},{:f},{:f},{:f},{:f},{:f}}},pose={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(
                a1, b1, c1, d1, e1, f1,  a2, b2, c2, d2, e2, f2)
        elif coordinateMode == 1:
            string = "Arc(joint={{{:f},{:f},{:f},{:f},{:f},{:f}}},joint={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(
                a1, b1, c1, d1, e1, f1,  a2, b2, c2, d2, e2, f2)
        else:
            print("coordinateMode  param  is wrong")
            return ""
        params = []
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        if a != -1:
            params.append('a={:d}'.format(a))
        if v != -1 and speed != -1:
            params.append('speed={:d}'.format(speed))
        elif speed != -1:
            params.append('speed={:d}'.format(speed))
        elif v != -1:
            params.append('v={:d}'.format(v))
        if cp != -1 and r != -1:
            params.append('r={:d}'.format(r))
        elif r != -1:
            params.append('r={:d}'.format(r))
        elif cp != -1:
            params.append('cp={:d}'.format(cp))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def Circle(self, a1, b1, c1, d1, e1, f1,  a2, b2, c2, d2, e2, f2, coordinateMode, count, user=-1, tool=-1, a=-1, v=-1, speed=-1, cp=-1, r=-1):
        """
        描述
        从当前位置进⾏整圆插补运动，运动指定圈数后重新回到当前位置。
        需要通过当前位置，P1，P2三个点确定⼀个整圆，因此当前位置不能在P1和P2确定的直线上，且
        三个点确定的整圆不能超出机械臂的运动范围。
        必选参数
        参数名 类型 说明
        P1 string 整圆中间点，⽀持关节变量或位姿变量
        P2 string 整圆结束点点，⽀持关节变量或位姿变量
        coordinateMode int  目标点的坐标值模式    0为pose方式  1为joint
        count int 进⾏整圆运动的圈数，取值范围：[1,999]。
        可选参数
        参数名  类型  说明
        user int ⽤⼾坐标系
        tool int ⼯具坐标系
        a int 执⾏该条指令时的机械臂运动加速度⽐例。取值范围：(0,100]
        v int 执⾏该条指令时的机械臂运动速度⽐例，与speed互斥。取值范围：(0,100]
        speed int执⾏该条指令时的机械臂运动⽬标速度，与v互斥，若同时存在以speed为
        准。取值范围：[1, 最⼤运动速度]，单位：mm/s
        cp int 平滑过渡⽐例，与r互斥。取值范围：[0,100]
        r int 平滑过渡半径，与cp互斥，若同时存在以r为准。单位：mm
        Description
        Move from the current position in a circle interpolated mode, and return to the current position after moving specified circles.
        As the circle needs to be determined through the current position, P1 and P2, the current position should not be in a straight line determined by P1 and P2, and the circle determined by the three points cannot exceed the motion range of the robot arm.
        Required parameter:
        Parameter name     Type     Description
        P1     string     Through point (joint variables or posture variables)
        P2     string     End point (joint variables or posture variables)
        coordinateMode     int      Coordinate mode of the target point, 0: pose, 1: joint
        count     int     Number of circles, range: [1,999].
        Optional parameter:
        Parameter name     Type     Description
        user     int     user coordinate system
        tool     int     tool coordinate system
        a     int     acceleration rate of the robot arm when executing this command. Range: (0,100].
        v     int     velocity rate of the robot arm when executing this command, incompatible with “speed”. Range: (0,100].
        speed     int     target speed of the robot arm when executing this command, incompatible with “v”. If both "speed" and "v” exist, speed takes precedence. Range: [0, maximum motion speed], unit: mm/s.
        cp     int     continuous path rate, incompatible with “r”. Range: [0,100].
        r     int     continuous path radius, incompatible with “cp”. If both "r" and "cp” exist, r takes precedence. Unit: mm.
        """
        string = ""
        if coordinateMode == 0:
            string = "Circle(pose={{{:f},{:f},{:f},{:f},{:f},{:f}}},pose={{{:f},{:f},{:f},{:f},{:f},{:f}}},{:d}".format(
                a1, b1, c1, d1, e1, f1,  a2, b2, c2, d2, e2, f2, count)
        elif coordinateMode == 1:
            string = "Circle(joint={{{:f},{:f},{:f},{:f},{:f},{:f}}},joint={{{:f},{:f},{:f},{:f},{:f},{:f}}},{:d}".format(
                a1, b1, c1, d1, e1, f1,  a2, b2, c2, d2, e2, f2, count)
        else:
            print("coordinateMode  param  is wrong")
            return ""
        params = []
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        if a != -1:
            params.append('a={:d}'.format(a))
        if v != -1 and speed != -1:
            params.append('speed={:d}'.format(speed))
        elif speed != -1:
            params.append('speed={:d}'.format(speed))
        elif v != -1:
            params.append('v={:d}'.format(v))
        if cp != -1 and r != -1:
            params.append('r={:d}'.format(r))
        elif r != -1:
            params.append('r={:d}'.format(r))
        elif cp != -1:
            params.append('cp={:d}'.format(cp))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def MoveJog(self, axis_id='', coordtype=-1, user=-1, tool=-1):
        """
        Joint motion
        axis_id: Joint motion axis, optional string value:
            J1+ J2+ J3+ J4+ J5+ J6+
            J1- J2- J3- J4- J5- J6- 
            X+ Y+ Z+ Rx+ Ry+ Rz+ 
            X- Y- Z- Rx- Ry- Rz-
        *dynParams: Parameter Settings（coord_type, user_index, tool_index）
                    coord_type: 1: User coordinate 2: tool coordinate (default value is 1)
                    user_index: user index is 0 ~ 9 (default value is 0)
                    tool_index: tool index is 0 ~ 9 (default value is 0)
        """
        string = "MoveJog({:s}".format(axis_id)
        params = []
        if coordtype != -1:
            params.append('coordtype={:d}'.format(coordtype))
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def GetStartPose(self, trace_name):
        """
        描述
        获取指定轨迹的第⼀个点位。
        必选参数
        参数名 类型 说明
        traceName string  轨迹⽂件名（含后缀）
        轨迹⽂件存放在/dobot/userdata/project/process/trajectory/
        如果名称包含中⽂，必须将发送端的编码⽅式设置为UTF-8，否则
        会导致中⽂接收异常
        Description
        Get the start point of the trajectory.
        Required parameter:
        Parameter name     Type     Description
        traceName     string     trajectory file name (with suffix)
        The trajectory file is stored in /dobot/userdata/project/process/trajectory/.
        If the name contains Chinese, the encoding of the sender must be set to UTF-8, otherwise
        it will cause an exception for receiving Chinese.
        """
        string = "GetStartPose({:s})".format(trace_name)
        return self.sendRecvMsg(string)

    def StartPath(self, trace_name, isConst=-1, multi=-1.0, user=-1, tool=-1):
        """
        描述
        根据指定的轨迹⽂件中的记录点位进⾏运动，复现录制的运动轨迹。
        下发轨迹复现指令成功后，⽤⼾可以通过RobotMode指令查询机械臂运⾏状态，
        ROBOT_MODE_RUNNING表⽰机器⼈在轨迹复现运⾏中，变成ROBOT_MODE_IDLE表⽰轨迹复现
        运⾏完成，ROBOT_MODE_ERROR表⽰报警。
        必选参数
        参数名 类型 说明
        traceName string
        轨迹⽂件名（含后缀）轨迹⽂件存放在/dobot/userdata/project/process/trajectory/
        如果名称包含中⽂，必须将发送端的编码⽅式设置为UTF-8，否则会导致中⽂接收异常
        可选参数
        参数名 类型 说明
        isConst int是否匀速复现。
           1表⽰匀速复现，机械臂会按照全局速率匀速复现轨迹；
           0表⽰按照轨迹录制时的原速复现，并可以使⽤multi参数等⽐缩放运
           动速度，此时机械臂的运动速度不受全局速率的影响。
        multi double 复现时的速度倍数，仅当isConst=0时有效；取值范围：[0.25, 2]，默认值为1
        user int  指定轨迹点位对应的⽤⼾坐标系索引，不指定时使⽤轨迹⽂件中记录的⽤⼾坐标系索引
        tool int 指定轨迹点位对应的⼯具坐标系索引，不指定时使⽤轨迹⽂件中记录的⼯具坐标系索引
        Description
        Move according to the recorded points in the specified trajectory file to play back the recorded trajectory.
        After the trajectory playback command is successfully delivered, you can check the robot status via RobotMode command.
        ROBOT_MODE_RUNNING: the robot is in trajectory playback, ROBOT_MODE_IDLE: trajectory playback is completed,
        ROBOT_MODE_ERROR: alarm.
        Required parameter:
        Parameter name     Type     Description
        traceName string
        trajectory file name (with suffix). The trajectory file is stored in /dobot/userdata/project/process/trajectory/.
        If the name contains Chinese, the encoding of the sender must be set to UTF-8, otherwise it will cause an exception for receiving Chinese.
        Optional parameter:
        Parameter name     Type     Description
        isConst     int     if or not to play back at a constant speed.
           1: the trajectory will be played back at the global rate at a uniform rate by the arm;
           0: the trajectory will be played back at the same speed as when it was recorded, and the motion speed can be scaled equivalently using the multi parameter, where the motion speed of the arm is not affected by the global rate.
        multi     double     Speed multiplier in playback, valid only when isConst=0. Range: [0.25, 2], 1 by default.
        user     int     User coordinate system index corresponding to the specified trajectory point (use the user coordinate system index recorded in the trajectory file if not specified).
        tool     int     tool coordinate system index corresponding to the specified trajectory point (use the tool coordinate system index recorded in the trajectory file if not specified).
        """
        string = "StartPath({:s}".format(trace_name)
        params = []
        if isConst != -1:
            params.append('isConst={:d}'.format(isConst))
        if multi != -1:
            params.append('multi={:f}'.format(multi))
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def RelMovJTool(self, offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz, user=-1, tool=-1, a=-1, v=-1, cp=-1):
        """
        描述
        沿⼯具坐标系进⾏相对运动，末端运动⽅式为关节运动。
        必选参数
        参数名 类型 说明
        offsetX double X轴⽅向偏移量，单位：mm
        offsetY double Y轴⽅向偏移量，单位：mm
        offsetZ double Z轴⽅向偏移量，单位：mm
        offsetRx double Rx轴⽅向偏移量，单位：度
        offsetRy double Ry轴⽅向偏移量，单位：度
        offsetRz double Rz轴⽅向偏移量，单位：度
        可选参数
        参数名 类型 说明
        user int ⽤⼾坐标系
        tool int ⼯具坐标系
        a int 执⾏该条指令时的机械臂运动加速度⽐例。取值范围：(0,100]
        v int 执⾏该条指令时的机械臂运动速度⽐例。取值范围：(0,100]
        cp int 平滑过渡⽐例。取值范围：[0,100]
        Description
        Perform relative motion along the tool coordinate system, and the end motion is joint motion.
        Required parameter:
        Parameter name     Type     Description
        offsetX     double     X-axis offset, unit: mm
        offsetY     double     Y-axis offset, unit: mm
        offsetZ     double     Z-axis offset, unit: mm
        offsetRx     double     Rx-axis offset, unit: °
        offsetRy     double     Ry-axis offset, unit: °
        offsetRz     double     Rz-axis offset, unit: °
        Optional parameter:
        Parameter name     Type     Description
        user     int     user coordinate system
        tool     int     tool coordinate system
        a     int     acceleration rate of the robot arm when executing this command. Range: (0,100].
        v     int     velocity rate of the robot arm when executing this command. Range: (0,100].
        cp     int     continuous path rate. Range: [0,100].
        """
        string = "RelMovJTool({:f},{:f},{:f},{:f},{:f},{:f}".format(
            offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz)
        params = []
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        if a != -1:
            params.append('a={:d}'.format(a))
        if v != -1:
            params.append('v={:d}'.format(v))
        if cp != -1:
            params.append('cp={:d}'.format(cp))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def RelMovLTool(self, offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz, user=-1, tool=-1, a=-1, v=-1, speed=-1, cp=-1, r=-1):
        """
        描述
        沿⼯具坐标系进⾏相对运动，末端运动⽅式为直线运动。
        此条指令为六轴机械臂特有。
        必选参数
        参数名 类型 说明
        offsetX double X轴⽅向偏移量，单位：mm
        offsetY double Y轴⽅向偏移量，单位：mm
        offsetZ double Z轴⽅向偏移量，单位：mm
        offsetRx double Rx轴⽅向偏移量，单位：度
        offsetRy double Ry轴⽅向偏移量，单位：度
        offsetRz double Rz轴⽅向偏移量，单位：度
        可选参数
        参数名  类型  说明
        user int ⽤⼾坐标系
        tool int ⼯具坐标系
        a int 执⾏该条指令时的机械臂运动加速度⽐例。取值范围：(0,100]
        v int 执⾏该条指令时的机械臂运动速度⽐例。取值范围：(0,100]
        speed int  执⾏该条指令时的机械臂运动⽬标速度，与v互斥，若同时存在以speed为
        准。取值范围：[1, 最⼤运动速度]，单位：mm/s
        cp int 平滑过渡⽐例，与r互斥。取值范围：[0,100]
        r int 平滑过渡半径，与cp互斥，若同时存在以r为准。单位：mm
        Description
        Perform relative motion along the tool coordinate system, and the end motion is linear motion.
        This command is for 6-axis robots.
        Required parameter:
        Parameter name     Type     Description
        offsetX     double     X-axis offset, unit: mm
        offsetY     double     Y-axis offset, unit: mm
        offsetZ     double     Z-axis offset, unit: mm
        offsetRx     double     Rx-axis offset, unit: °
        offsetRy     double     Ry-axis offset, unit: °
        offsetRz     double     Rz-axis offset, unit: °
        Optional parameter:
        Parameter name     Type     Description
        user     int     user coordinate system
        tool     int     tool coordinate system
        a     int     acceleration rate of the robot arm when executing this command. Range: (0,100].
        v     int     velocity rate of the robot arm when executing this command. Range: (0,100].
        speed     int     target speed of the robot arm when executing this command, incompatible with “v”. If both "speed" and "v” exist, speed takes precedence. Range: [0, maximum motion speed], unit: mm/s.
        cp     int     continuous path rate, incompatible with “r”. Range: [0,100].
        r     int     continuous path radius, incompatible with “cp”. If both "r" and "cp” exist, r takes precedence. Unit: mm.
        """
        string = "RelMovLTool({:f},{:f},{:f},{:f},{:f},{:f}".format(
            offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz)
        params = []
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        if a != -1:
            params.append('a={:d}'.format(a))
        if v != -1 and speed != -1:
            params.append('speed={:d}'.format(speed))
        elif speed != -1:
            params.append('speed={:d}'.format(speed))
        elif v != -1:
            params.append('v={:d}'.format(v))
        if cp != -1 and r != -1:
            params.append('r={:d}'.format(r))
        elif r != -1:
            params.append('r={:d}'.format(r))
        elif cp != -1:
            params.append('cp={:d}'.format(cp))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def RelMovJUser(self, offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz, user=-1, tool=-1, a=-1, v=-1, cp=-1):
        """
        描述
        沿⽤⼾坐标系进⾏相对运动，末端运动⽅式为关节运动。
        必选参数
        参数名 类型 说明
        offsetX double X轴⽅向偏移量，单位：mm
        offsetY double Y轴⽅向偏移量，单位：mm
        offsetZ double Z轴⽅向偏移量，单位：mm
        offsetRx double Rx轴偏移量，单位：度
        offsetRy double Ry轴偏移量，单位：度
        offsetRz double Rz轴偏移量，单位：度
        可选参数
        参数名 类型 说明
        user int ⽤⼾坐标系
        tool int ⼯具坐标系
        a int 执⾏该条指令时的机械臂运动加速度⽐例。取值范围：(0,100]
        v int 执⾏该条指令时的机械臂运动速度⽐例。取值范围：(0,100]
        cp int 平滑过渡⽐例。取值范围：[0,100]
        Description
        Perform relative motion along the user coordinate system, and the end motion is joint motion.
        Required parameter:
        Parameter name     Type     Description
        offsetX     double     X-axis offset, unit: mm
        offsetY     double     Y-axis offset, unit: mm
        offsetZ     double     Z-axis offset, unit: mm
        offsetRx     double     Rx-axis offset, unit: °
        offsetRy     double     Ry-axis offset, unit: °
        offsetRz     double     Rz-axis offset, unit: °
        Optional parameter:
        Parameter name     Type     Description
        user     int     user coordinate system
        tool     int     tool coordinate system
        a     int     acceleration rate of the robot arm when executing this command. Range: (0,100].
        v     int     velocity rate of the robot arm when executing this command. Range: (0,100].
        cp     int     continuous path rate. Range: [0,100].
        """
        string = "RelMovJUser({:f},{:f},{:f},{:f},{:f},{:f}".format(
            offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz)
        params = []
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        if a != -1:
            params.append('a={:d}'.format(a))
        if v != -1:
            params.append('v={:d}'.format(v))
        if cp != -1:
            params.append('cp={:d}'.format(cp))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def RelMovLUser(self, offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz, user=-1, tool=-1, a=-1, v=-1, speed=-1, cp=-1, r=-1):
        """
        描述
        沿⽤⼾坐标系进⾏相对运动，末端运动⽅式为直线运动。
        必选参数
        参数名 类型 说明
        offsetX double X轴⽅向偏移量，单位：mm
        offsetY double Y轴⽅向偏移量，单位：mm
        offsetZ double Z轴⽅向偏移量，单位：mm
        offsetRx double Rx轴偏移量，单位：度
        offsetRy double Ry轴偏移量，单位：度
        offsetRz double Rz轴偏移量，单位：度
        可选参数
        参数名  类型说明
        user int ⽤⼾坐标系
        tool int ⼯具坐标系
        a int 执⾏该条指令时的机械臂运动加速度⽐例。取值范围：(0,100]
        v int 执⾏该条指令时的机械臂运动速度⽐例。取值范围：(0,100]
        speed int 执⾏该条指令时的机械臂运动⽬标速度，与v互斥，若同时存在以speed为
        准。取值范围：[1, 最⼤运动速度]，单位：mm/s
        cp int 平滑过渡⽐例，与r互斥。取值范围：[0,100]
        r int 平滑过渡半径，与cp互斥，若同时存在以r为准。单位：mm
        Description
        Perform relative motion along the user coordinate system, and the end motion is linear motion.
        Required parameter:
        Parameter name     Type     Description
        offsetX     double     X-axis offset, unit: mm
        offsetY     double     Y-axis offset, unit: mm
        offsetZ     double     Z-axis offset, unit: mm
        offsetRx     double     Rx-axis offset, unit: °
        offsetRy     double     Ry-axis offset, unit: °
        offsetRz     double     Rz-axis offset, unit: °
        Optional parameter:
        Parameter name     Type     Description
        user     int     user coordinate system
        tool     int     tool coordinate system
        a     int     acceleration rate of the robot arm when executing this command. Range: (0,100].
        v     int     velocity rate of the robot arm when executing this command. Range: (0,100].
        speed     int     target speed of the robot arm when executing this command, incompatible with “v”. If both "speed" and "v” exist, speed takes precedence. Range: [0, maximum motion speed], unit: mm/s.
        cp     int     continuous path rate, incompatible with “r”. Range: [0,100].
        r     int     continuous path radius, incompatible with “cp”. If both "r" and "cp” exist, r takes precedence. Unit: mm.
        """
        string = "RelMovLUser({:f},{:f},{:f},{:f},{:f},{:f}".format(
            offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz)
        params = []
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        if a != -1:
            params.append('a={:d}'.format(a))
        if v != -1 and speed != -1:
            params.append('speed={:d}'.format(speed))
        elif speed != -1:
            params.append('speed={:d}'.format(speed))
        elif v != -1:
            params.append('v={:d}'.format(v))
        if cp != -1 and r != -1:
            params.append('r={:d}'.format(r))
        elif r != -1:
            params.append('r={:d}'.format(r))
        elif cp != -1:
            params.append('cp={:d}'.format(cp))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def RelJointMovJ(self, offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz, a=-1, v=-1, cp=-1):
        """
        描述
        沿关节坐标系进⾏相对运动，末端运动⽅式为关节运动。
        必选参数
        参数名 类型 说明
        offset1 double J1轴偏移量，单位：度
        offset2 double J2轴偏移量，单位：度
        offset3 double J3轴偏移量，单位：度
        offset4 double J4轴偏移量，单位：度
        offset5 double J5轴偏移量，单位：度
        offset6 double J6轴偏移量，单位：度
        可选参数
        参数名 类型 说明
        a int 执⾏该条指令时的机械臂运动加速度⽐例。取值范围：(0,100]
        v int 执⾏该条指令时的机械臂运动速度⽐例。取值范围：(0,100]
        cp int 平滑过渡⽐例。取值范围：[0,100]
        Description
        Perform relative motion along the joint coordinate system, and the end motion is joint motion.
        Required parameter:
        Parameter name     Type     Description
        offset1     double     J1-axis offset, unit: °
        offset2     double     J2-axis offset, unit: °
        offset3     double     J3-axis offset, unit: °
        offset4     double     J4-axis offset, unit: °
        offset5     double     J5-axis offset, unit: °
        offset6     double     J6-axis offset, unit: °
        Optional parameter:
        Parameter name     Type     Description
        a     int     acceleration rate of the robot arm when executing this command. Range: (0,100].
        v     int     velocity rate of the robot arm when executing this command. Range: (0,100].
        cp     int     continuous path rate. Range: [0,100].
        """
        string = "RelJointMovJ({:f},{:f},{:f},{:f},{:f},{:f}".format(
            offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz)
        params = []
        if a != -1:
            params.append('a={:d}'.format(a))
        if v != -1:
            params.append('v={:d}'.format(v))
        if cp != -1:
            params.append('cp={:d}'.format(cp))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def GetCurrentCommandID(self):
        """
        獲取當前執行指令的演算法佇列 ID，可以用於判斷當前機器人執行到了哪一條指令。
        Get the algorithm queue ID of the currently executed command, which can be used to judge which command is currently being executed by the robot.
        """
        string = "GetCurrentCommandID()"
        return self.sendRecvMsg(string)

    ###################################460 新增#############################
    
    ## 軌跡恢復指令
    def SetResumeOffset(self, distance):
        """
       該指令僅用於焊接工藝。設定軌跡恢復的目標點位相對暫停時的點位沿焊縫回退的距離
        """
        string = "SetResumeOffset({:f})".format(distance)
        return self.sendRecvMsg(string)
    
    def PathRecovery(self):
        """
        開始軌跡恢復：工程暫停後，控制機器人回到暫停時的位姿。
        """
        string = "PathRecovery()"
        return self.sendRecvMsg(string)

    def PathRecoveryStop(self):
        """
        軌跡恢復的過程中停止機器人。
        """
        string = "PathRecoveryStop()"
        return self.sendRecvMsg(string)

    def PathRecoveryStatus(self):
        """
        查詢軌跡恢復的狀態。
        """
        string = "PathRecoveryStatus()"
        return self.sendRecvMsg(string)
    
    ## 日誌匯出指令
    def LogExportUSB(self, range):
        """
       將機器人日誌匯出至插在機器人控制櫃 USB 介面的隨身碟根目錄。
       匯出範圍。
        0   匯出 logs/all 和 logs/user 資料夾的內容。
        1   匯出 logs 資料夾所有內容。
        """
        string = "LogExportUSB({:d})".format(range)
        return self.sendRecvMsg(string)
    
    def GetExportStatus(self):
        """
        獲取日誌匯出的狀態。
        其中 status 表示日誌匯出狀態。
        0：未開始匯出
        1：匯出中
        2：匯出完成
        3：匯出失敗，找不到隨身碟
        4：匯出失敗，隨身碟空間不足
        5：匯出失敗，匯出過程中隨身碟被拔出
        匯出完成和匯出失敗的狀態會保持到下次使用者使用匯出功能
        """
        string = "GetExportStatus()"
        return self.sendRecvMsg(string)

    ## 力控指令
    def EnableFTSensor(self, status):
        """
       開啟/關閉力傳感器。
        """
        string = "EnableFTSensor({:d})".format(status)
        return self.sendRecvMsg(string)

    def SixForceHome(self):
        """
        將力傳感器當前數值置 0，即以傳感器當前受力狀態作為零點。
        """
        string = "SixForceHome()"
        return self.sendRecvMsg(string)
    
    def GetForce(self, tool = -1):
        """
        獲取力傳感器當前數值。
        tool int 用於指定獲取數值時參考的工具座標系，取值範圍：[0,50]。
        不指定時使用全域工具座標系
        """
        if tool == -1:
            string = "GetForce()"
        else:
            string = "GetForce({:d})".format(tool)
        return self.sendRecvMsg(string)

    def ForceDriveMode(self, x, y, z, rx, ry, rz, user=-1):
        """
        指定可拖拽的方向並進入力控拖拽模式。
        {x,y,z,rx,ry,rz} string
        用於指定可拖拽的方向。
        0 代表該方向不能拖拽，1 代表該方向可以拖拽。
        例：
        {1,1,1,1,1,1} 表示機械臂可在各軸方向上自由拖動
        {1,1,1,0,0,0} 表示機械臂僅可在 XYZ 軸方向上拖動
        {0,0,0,1,1,1} 表示機械臂僅可在 RxRyRz 軸方向上旋轉
        """
        string = ""
        string = "ForceDriveMode("+"{"+"{:d},{:d},{:d},{:d},{:d},{:d}".format(x,y,z,rx,ry,rz)+"}"
        if user != -1:
            string = string + ',{:d}'.format(user)
        string = string + ')'
        return self.sendRecvMsg(string)
    
    def ForceDriveSpeed(self, speed):
        """
        設定力控拖拽速度比例。
        speed int 力控拖拽速度比例，取值範圍：[1,100]。
        """
        string = "ForceDriveSpeed({:d})".format(speed)
        return self.sendRecvMsg(string)
    
    def FCForceMode(self, x, y, z, rx, ry, rz, fx,fy,fz,frx,fry,frz, reference=-1, user=-1,tool=-1):
        """
        以使用者指定的配置參數開啟力控。
        {x,y,z,rx,ry,rz}
            開啟/關閉笛卡爾空間某個方向的力控調節。
            0 表示關閉該方向的力控。
            1 表示開啟該方向的力控。
        {fx,fy,fz,frx,fry,frz}
            目標力：是工具末端與作用對象之間接觸力的目標值，是一種模擬力，可以由使用者自行設定；目標力方向分別對應笛卡爾空間的 {x,y,z,rx,ry,rz} 方向。
            位移方向的目標力範圍 [-200,200]，單位 N；姿態方向的目標力範圍 [-12,12]，單位 N/m。
            目標力為 0 時處於柔順模式，柔順模式與力控拖動類似。
        如果某個方向未開啟力控調節，則該方向的目標力也不會生效。
        reference
            格式為 "reference=value"。value 表示參考座標系，預設參考工具座標系。
            reference=0 表示參考工具座標系，即沿工具座標系進行力控調節。
            reference=1 表示參考使用者座標系，即沿使用者座標系進行力控調節。
        user
            格式為 "user=index"，index 為已標定的使用者座標系索引。取值範圍：[0,50]。
        tool
            格式為 "tool=index"，index 為已標定的工具座標系索引。取值範圍：[0,50]。
        """
        string = ""
        string = "FCForceMode("+"{"+"{:d},{:d},{:d},{:d},{:d},{:d}".format(x,y,z,rx,ry,rz)+"},"+"{"+"{:d},{:d},{:d},{:d},{:d},{:d}".format(fx,fy,fz,frx,fry,frz)+"}"
        params = []
        if reference != -1:
            params.append('reference={:d}'.format(reference))
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        for ii in params:
            string = string + ','+ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def FCSetDeviation(self, x, y, z, rx, ry, rz, controltype=-1):
        """
        設定力控模式下的位移和姿態偏差，若力控過程中恆力偏移了較大的距離，機器人會進行相應處理。
        x、y、z
        代表力控模式下的位移偏差，單位為 mm。取值範圍：(0,1000]，預設值 100mm。
        rx、ry、rz
        代表力控模式下的姿態偏差，單位為度。取值範圍：(0,360]，預設值 36 度。
        controltype
        表示力控過程中超過規定閾值時，機械臂的處理方式。
        0：超過閾值時，機械臂報警（預設值）。
        1：超過閾值時，機械臂停止搜尋而在原有軌跡上繼續運動。
        """
        string = ""
        string = "FCSetDeviation("+"{"+"{:d},{:d},{:d},{:d},{:d},{:d}".format(x,y,z,rx,ry,rz)+"}"
        if controltype != -1:
            string = string + ',{:d}'.format(controltype)
        string = string + ')'
        return self.sendRecvMsg(string)
    
    def FCSetForceLimit(self, x, y, z, rx, ry, rz):
        """
        設定各方向的最大力限制（該設定對所有方向均生效，包含未啟用力控的方向）。
        """
        string = ""
        string = "FCSetForceLimit("+"{:d},{:d},{:d},{:d},{:d},{:d}".format(x,y,z,rx,ry,rz)
        string = string + ')'
        return self.sendRecvMsg(string)
    
    def FCSetMass(self, x, y, z, rx, ry, rz):
        """
        設定力控模式下各方向的慣性係數。
        """
        string = ""
        string = "FCSetMass("+"{:d},{:d},{:d},{:d},{:d},{:d}".format(x,y,z,rx,ry,rz)
        string = string + ')'
        return self.sendRecvMsg(string)
    
    def FCSetStiffness(self, x, y, z, rx, ry, rz):
        """
        設定力控模式下各方向的彈性係數。
        """
        string = ""
        string = "FCSetStiffness("+"{:d},{:d},{:d},{:d},{:d},{:d}".format(x,y,z,rx,ry,rz)
        string = string + ')'
        return self.sendRecvMsg(string)
    
    def FCSetDamping(self, x, y, z, rx, ry, rz):
        """
        設定力控模式下各方向的阻尼係數。
        """
        string = ""
        string = "FCSetDamping("+"{:d},{:d},{:d},{:d},{:d},{:d}".format(x,y,z,rx,ry,rz)
        string = string + ')'
        return self.sendRecvMsg(string)

    def FCOff(self):
        """
        退出力控模式，與 FCForceMode 配合使用，兩者之間的運動指令都會進行力的柔順控制。
        """
        string = "FCOff()"
        return self.sendRecvMsg(string)

    def FCSetForceSpeedLimit(self, x, y, z, rx, ry, rz):
        """
        設定各方向的力控調節速度。力控速度上限較小時，力控調節速度較慢，適合低速平緩的接觸面。
        力控速度上限較大時，力控調節速度快，適合高速力控應用。需要根據具體的應用場景進行調整。
        """
        string = ""
        string = "FCSetForceSpeedLimit("+"{:d},{:d},{:d},{:d},{:d},{:d}".format(x,y,z,rx,ry,rz)
        string = string + ')'
        return self.sendRecvMsg(string)
    
    def FCSetForce(self, x, y, z, rx, ry, rz):
        """
        即時調整各方向的恆力設定。
        """
        string = ""
        string = "FCSetForce("+"{:d},{:d},{:d},{:d},{:d},{:d}".format(x,y,z,rx,ry,rz)
        string = string + ')'
        return self.sendRecvMsg(string)
    
    def RequestControl(self):
        """
        Request control of the robot.
        Note: This function sends a request for the control of the robot, which may be approved or denied.
        """
        string = "RequestControl()"
        return self.sendRecvMsg(string)
    
    ## 新增運動指令

    def RelPointTool(self, coordinateMode,a1, b1, c1, d1, e1, f1, x, y, z, rx, ry, rz):
        """
        沿工具座標系笛卡爾點偏移。
        """
        string = ""
        if coordinateMode == 0:
            string = "RelPointTool(pose={{{:f},{:f},{:f},{:f},{:f},{:f}}},".format(
                a1, b1, c1, d1, e1, f1)
        elif coordinateMode == 1:
            string = "RelPointTool(joint={{{:f},{:f},{:f},{:f},{:f},{:f}}},".format(
                a1, b1, c1, d1, e1, f1)
        string = string + "{"+"{:f},{:f},{:f},{:f},{:f},{:f}".format(x,y,z,rx,ry,rz)+"}"
        string = string + ')'
        return self.sendRecvMsg(string)
    
    def RelPointUser(self,coordinateMode,a1, b1, c1, d1, e1, f1, x, y, z, rx, ry, rz):
        """
        沿使用者座標系笛卡爾點偏移。
        """
        string = ""
        string = ""
        if coordinateMode == 0:
            string = "RelPointUser(pose={{{:f},{:f},{:f},{:f},{:f},{:f}}},".format(
                a1, b1, c1, d1, e1, f1)
        elif coordinateMode == 1:
            string = "RelPointUser(joint={{{:f},{:f},{:f},{:f},{:f},{:f}}},".format(
                a1, b1, c1, d1, e1, f1)
        string =string + "{"+"{:f},{:f},{:f},{:f},{:f},{:f}".format(x,y,z,rx,ry,rz)+"}"
        string = string + ')'
        return self.sendRecvMsg(string)

    def RelJoint(self, j1, j2, j3, j4, j5, j6, offset1, offset2, offset3, offset4, offset5, offset6):
        """
        RelJoint command
        """
        string = "RelJoint({:f},{:f},{:f},{:f},{:f},{:f},{{{:f},{:f},{:f},{:f},{:f},{:f}}})".format(
            j1, j2, j3, j4, j5, j6, offset1, offset2, offset3, offset4, offset5, offset6)
        return self.sendRecvMsg(string)
    
    def GetError(self, language="zh_cn"):
        """
        获取机器人报警信息
        参数:
        language: 语言设置，支持的值:
                 "zh_cn" - 简体中文
                 "zh_hant" - 繁体中文  
                 "en" - 英语
                 "ja" - 日语
                 "de" - 德语
                 "vi" - 越南语
                 "es" - 西班牙语
                 "fr" - 法语
                 "ko" - 韩语
                 "ru" - 俄语
        返回:
        dict: 包含报警信息的字典，格式如下:
        {
            "errMsg": [
                {
                    "id": xxx,
                    "level": xxx,
                    "description": "xxx",
                    "solution": "xxx",
                    "mode": "xxx",
                    "date": "xxxx",
                    "time": "xxxx"
                }
            ]
        }
        """
        try:
            # 首先设置语言
            language_url = f"http://{self.ip}:22000/interface/language"
            language_data = {"type": language}
            
            # 发送POST请求设置语言
            response = requests.post(language_url, json=language_data, timeout=5)
            if response.status_code != 200:
                print(f"设置语言失败: HTTP {response.status_code}")
            
            # 获取报警信息
            alarm_url = f"http://{self.ip}:22000/protocol/getAlarm"
            response = requests.get(alarm_url, timeout=5)
            
            if response.status_code == 200:
                return response.json()
            else:
                print(f"获取报警信息失败: HTTP {response.status_code}")
                return {"errMsg": []}
                
        except requests.exceptions.RequestException as e:
            print(f"HTTP请求异常: {e}")
            return {"errMsg": []}
        except json.JSONDecodeError as e:
            print(f"JSON解析异常: {e}")
            return {"errMsg": []}
        except Exception as e:
            print(f"获取报警信息时发生未知错误: {e}")
            return {"errMsg": []}

    def ArcIO(self, a1, b1, c1, d1, e1, f1, a2, b2, c2, d2, e2, f2, coordinateMode, *io_params, user=-1, tool=-1, a=-1, v=-1, speed=-1, cp=-1, r=-1, mode=-1):
        """
        圆弧运动过程中并行设置数字输出端口的状态，可设置多组。
        """
        string = ""
        if coordinateMode == 0:
            string = "ArcIO(pose={{{:f},{:f},{:f},{:f},{:f},{:f}}},pose={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(
                a1, b1, c1, d1, e1, f1, a2, b2, c2, d2, e2, f2)
        elif coordinateMode == 1:
            string = "ArcIO(joint={{{:f},{:f},{:f},{:f},{:f},{:f}}},joint={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(
                a1, b1, c1, d1, e1, f1, a2, b2, c2, d2, e2, f2)
        else:
            print("coordinateMode  param  is wrong")
            return ""
        
        for io_param in io_params:
            if isinstance(io_param, (list, tuple)) and len(io_param) == 4:
                string += ",{{{:d},{:d},{:d},{:d}}}".format(*io_param)
            else:
                 print("io_param format is wrong")

        params = []
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        if a != -1:
            params.append('a={:d}'.format(a))
        if v != -1 and speed != -1:
            params.append('speed={:d}'.format(speed))
        elif speed != -1:
            params.append('speed={:d}'.format(speed))
        elif v != -1:
            params.append('v={:d}'.format(v))
        if cp != -1 and r != -1:
            params.append('r={:d}'.format(r))
        elif r != -1:
            params.append('r={:d}'.format(r))
        elif cp != -1:
            params.append('cp={:d}'.format(cp))
        if mode != -1:
            params.append('mode={:d}'.format(mode))
        
        for ii in params:
            string += ',' + ii
        string += ')'
        return self.sendRecvMsg(string)

    def ArcTrackStart(self):
        return self.sendRecvMsg("ArcTrackStart()")

    def ArcTrackParams(self, sampleTime, coordinateType, upDownCompensationMin, upDownCompensationMax, upDownCompensationOffset, leftRightCompensationMin, leftRightCompensationMax, leftRightCompensationOffset):
        string = "ArcTrackParams({:d},{:d},{:f},{:f},{:f},{:f},{:f},{:f})".format(
            sampleTime, coordinateType, upDownCompensationMin, upDownCompensationMax, upDownCompensationOffset, leftRightCompensationMin, leftRightCompensationMax, leftRightCompensationOffset)
        return self.sendRecvMsg(string)

    def ArcTrackEnd(self):
        return self.sendRecvMsg("ArcTrackEnd()")

    def CheckMovC(self, j1a, j2a, j3a, j4a, j5a, j6a, j1b, j2b, j3b, j4b, j5b, j6b, j1c, j2c, j3c, j4c, j5c, j6c, user=-1, tool=-1, a=-1, v=-1, cp=-1):
        string = "CheckMovC(joint={{{:f},{:f},{:f},{:f},{:f},{:f}}},joint={{{:f},{:f},{:f},{:f},{:f},{:f}}},joint={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(
            j1a, j2a, j3a, j4a, j5a, j6a, j1b, j2b, j3b, j4b, j5b, j6b, j1c, j2c, j3c, j4c, j5c, j6c)
        params = []
        if user != -1: params.append('user={:d}'.format(user))
        if tool != -1: params.append('tool={:d}'.format(tool))
        if a != -1: params.append('a={:d}'.format(a))
        if v != -1: params.append('v={:d}'.format(v))
        if cp != -1: params.append('cp={:d}'.format(cp))
        if params: string += "," + ",".join(params)
        string += ")"
        return self.sendRecvMsg(string)

    def CheckMovJ(self, j1a, j2a, j3a, j4a, j5a, j6a, j1b, j2b, j3b, j4b, j5b, j6b, user=-1, tool=-1, a=-1, v=-1, cp=-1):
        string = "CheckMovJ(joint={{{:f},{:f},{:f},{:f},{:f},{:f}}},joint={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(
            j1a, j2a, j3a, j4a, j5a, j6a, j1b, j2b, j3b, j4b, j5b, j6b)
        params = []
        if user != -1: params.append('user={:d}'.format(user))
        if tool != -1: params.append('tool={:d}'.format(tool))
        if a != -1: params.append('a={:d}'.format(a))
        if v != -1: params.append('v={:d}'.format(v))
        if cp != -1: params.append('cp={:d}'.format(cp))
        if params: string += "," + ",".join(params)
        string += ")"
        return self.sendRecvMsg(string)

    def CheckOddMovC(self, j1a, j2a, j3a, j4a, j5a, j6a, j1b, j2b, j3b, j4b, j5b, j6b, j1c, j2c, j3c, j4c, j5c, j6c, user=-1, tool=-1, a=-1, v=-1, cp=-1):
        string = "CheckOddMovC(joint={{{:f},{:f},{:f},{:f},{:f},{:f}}},joint={{{:f},{:f},{:f},{:f},{:f},{:f}}},joint={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(
            j1a, j2a, j3a, j4a, j5a, j6a, j1b, j2b, j3b, j4b, j5b, j6b, j1c, j2c, j3c, j4c, j5c, j6c)
        params = []
        if user != -1: params.append('user={:d}'.format(user))
        if tool != -1: params.append('tool={:d}'.format(tool))
        if a != -1: params.append('a={:d}'.format(a))
        if v != -1: params.append('v={:d}'.format(v))
        if cp != -1: params.append('cp={:d}'.format(cp))
        if params: string += "," + ",".join(params)
        string += ")"
        return self.sendRecvMsg(string)

    def CheckOddMovJ(self, j1a, j2a, j3a, j4a, j5a, j6a, j1b, j2b, j3b, j4b, j5b, j6b, user=-1, tool=-1, a=-1, v=-1, cp=-1):
        string = "CheckOddMovJ(joint={{{:f},{:f},{:f},{:f},{:f},{:f}}},joint={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(
            j1a, j2a, j3a, j4a, j5a, j6a, j1b, j2b, j3b, j4b, j5b, j6b)
        params = []
        if user != -1: params.append('user={:d}'.format(user))
        if tool != -1: params.append('tool={:d}'.format(tool))
        if a != -1: params.append('a={:d}'.format(a))
        if v != -1: params.append('v={:d}'.format(v))
        if cp != -1: params.append('cp={:d}'.format(cp))
        if params: string += "," + ",".join(params)
        string += ")"
        return self.sendRecvMsg(string)

    def CheckOddMovL(self, j1a, j2a, j3a, j4a, j5a, j6a, j1b, j2b, j3b, j4b, j5b, j6b, user=-1, tool=-1, a=-1, v=-1, cp=-1):
        string = "CheckOddMovL(joint={{{:f},{:f},{:f},{:f},{:f},{:f}}},joint={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(
            j1a, j2a, j3a, j4a, j5a, j6a, j1b, j2b, j3b, j4b, j5b, j6b)
        params = []
        if user != -1: params.append('user={:d}'.format(user))
        if tool != -1: params.append('tool={:d}'.format(tool))
        if a != -1: params.append('a={:d}'.format(a))
        if v != -1: params.append('v={:d}'.format(v))
        if cp != -1: params.append('cp={:d}'.format(cp))
        if params: string += "," + ",".join(params)
        string += ")"
        return self.sendRecvMsg(string)

    def CnvInit(self, index):
        """
        CnvInit command
        """
        string = "CnvInit({:d})".format(index)
        return self.sendRecvMsg(string)

    def CnvMovL(self, j1, j2, j3, j4, j5, j6, user=-1, tool=-1, a=-1, v=-1, cp=-1, r=-1):
        string = "CnvMovL(pose={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(
            j1, j2, j3, j4, j5, j6)
        params = []
        if user != -1: params.append('user={:d}'.format(user))
        if tool != -1: params.append('tool={:d}'.format(tool))
        if a != -1: params.append('a={:d}'.format(a))
        if v != -1: params.append('v={:d}'.format(v))
        if cp != -1: params.append('cp={:d}'.format(cp))
        if r != -1: params.append('r={:d}'.format(r))
        if params: string += "," + ",".join(params)
        string += ")"
        return self.sendRecvMsg(string)

    def CnvMovC(self, j1a, j2a, j3a, j4a, j5a, j6a, j1b, j2b, j3b, j4b, j5b, j6b, user=-1, tool=-1, a=-1, v=-1, cp=-1, r=-1, mode=1):
        string = "CnvMovC(pose={{{:f},{:f},{:f},{:f},{:f},{:f}}},pose={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(
            j1a, j2a, j3a, j4a, j5a, j6a, j1b, j2b, j3b, j4b, j5b, j6b)
        params = []
        if user != -1: params.append('user={:d}'.format(user))
        if tool != -1: params.append('tool={:d}'.format(tool))
        if a != -1: params.append('a={:d}'.format(a))
        if v != -1: params.append('v={:d}'.format(v))
        if cp != -1: params.append('cp={:d}'.format(cp))
        if r != -1: params.append('r={:d}'.format(r))
        if mode != 1: params.append('mode={:d}'.format(mode))
        if params: string += "," + ",".join(params)
        string += ")"
        return self.sendRecvMsg(string)

    def CreateTray(self, *args, **kwargs):
        """
        CreateTray command
        Due to missing documentation on exact parameters, this function uses dynamic arguments.
        Example: CreateTray(rows=3, cols=4, ...)
        """
        return self.sendRecvMsg(self._build_cmd("CreateTray", *args, **kwargs))

    def EndRTOffset(self):
        return self.sendRecvMsg("EndRTOffset()")

    def StartRTOffset(self):
        """
        StartRTOffset command
        """
        return self.sendRecvMsg("StartRTOffset()")

    def FCCollisionSwitch(self, enable):
        return self.sendRecvMsg("FCCollisionSwitch(enable={:d})".format(enable))

    def SetFCCollision(self, force, torque):
        return self.sendRecvMsg("SetFCCollision({:f},{:f})".format(force, torque))

    def GetCnvObject(self, objId):
        return self.sendRecvMsg("GetCnvObject({:d})".format(objId))

    def DOGroupDEC(self, group, value):
        return self.sendRecvMsg("DOGroupDEC({:d},{:d})".format(group, value))

    def GetDOGroupDEC(self, group, value):
        return self.sendRecvMsg("GetDOGroupDEC({:d},{:d})".format(group, value))

    def DIGroupDEC(self, group, value):
        return self.sendRecvMsg("DIGroupDEC({:d},{:d})".format(group, value))

    def InverseSolution(self, a1, b1, c1, d1, e1, f1, user=-1, tool=-1, isJoint=0):
        """
        InverseSolution command
        """
        string = "InverseSolution(pose={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(
            a1, b1, c1, d1, e1, f1)
        
        params = []
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        if isJoint != 0:
            params.append('isJoint={:d}'.format(isJoint))
            
        for ii in params:
            string += ',' + ii
        string += ')'
        return self.sendRecvMsg(string)

    def MoveL(self, a1, b1, c1, d1, e1, f1, user=-1, tool=-1, a=-1, v=-1, speed=-1, cp=-1, r=-1):
        """
        MoveL command
        """
        string = "MoveL(pose={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(
            a1, b1, c1, d1, e1, f1)
        
        params = []
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        if a != -1:
            params.append('a={:d}'.format(a))
        if v != -1 and speed != -1:
            params.append('speed={:d}'.format(speed))
        elif speed != -1:
            params.append('speed={:d}'.format(speed))
        elif v != -1:
            params.append('v={:d}'.format(v))
        if cp != -1 and r != -1:
            params.append('r={:d}'.format(r))
        elif r != -1:
            params.append('r={:d}'.format(r))
        elif cp != -1:
            params.append('cp={:d}'.format(cp))
            
        for ii in params:
            string += ',' + ii
        string += ')'
        return self.sendRecvMsg(string)

    def MovS(self, file=None, coordinateMode=-1, points=None, user=-1, tool=-1, v=-1, speed=-1, a=-1, freq=-1):
        """
        MovS command
        """
        string = "MovS("
        if file is not None:
             string += "file={:s}".format(file)
        elif points is not None and coordinateMode != -1:
             # points should be a list of tuples/lists
             pts_str = []
             for pt in points:
                 if coordinateMode == 0:
                     pts_str.append("pose={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(*pt))
                 elif coordinateMode == 1:
                     pts_str.append("joint={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(*pt))
             string += ",".join(pts_str)
        else:
             print("MovS param is wrong")
             return ""
        
        params = []
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        if v != -1 and speed != -1:
            params.append('speed={:d}'.format(speed))
        elif speed != -1:
            params.append('speed={:d}'.format(speed))
        elif v != -1:
            params.append('v={:d}'.format(v))
        if a != -1:
             params.append('a={:d}'.format(a))
        if freq != -1:
             params.append('freq={:d}'.format(freq))
             
        if len(params) > 0:
             if file is not None or (points is not None and len(points) > 0):
                  string += ","
             string += ",".join(params)
             
        string += ")"
        return self.sendRecvMsg(string)

    def OffsetPara(self, x, y, z, rx, ry, rz):
        """
        OffsetPara command
        """
        string = "OffsetPara({:f},{:f},{:f},{:f},{:f},{:f})".format(x, y, z, rx, ry, rz)
        return self.sendRecvMsg(string)


    def GetTrayPoint(self, *args, **kwargs):
        """
        GetTrayPoint command
        Due to missing documentation on exact parameters, this function uses dynamic arguments.
        Example: GetTrayPoint(trayName)
        """
        return self.sendRecvMsg(self._build_cmd("GetTrayPoint", *args, **kwargs))

    def ResetRobot(self):
        return self.sendRecvMsg("ResetRobot()")

    def RunTo(self, a1, b1, c1, d1, e1, f1, moveType, user=-1, tool=-1, a=-1, v=-1):
        """
        RunTo command
        """
        string = ""
        if moveType == 0:
            string = "RunTo(pose={{{:f},{:f},{:f},{:f},{:f},{:f}}},moveType=0".format(
                a1, b1, c1, d1, e1, f1)
        elif moveType == 1:
            string = "RunTo(joint={{{:f},{:f},{:f},{:f},{:f},{:f}}},moveType=1".format(
                a1, b1, c1, d1, e1, f1)
        else:
             print("moveType param is wrong")
             return ""
        
        params = []
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        if a != -1:
            params.append('a={:d}'.format(a))
        if v != -1:
            params.append('v={:d}'.format(v))
        
        for ii in params:
            string += ',' + ii
        string += ')'
        return self.sendRecvMsg(string)

    def SetArcTrackOffset(self, offsetX, offsetY, offsetZ, offsetRx, offsetRy, offsetRz):
        string = "SetArcTrackOffset({{{:f},{:f},{:f},{:f},{:f},{:f}}})".format(
            offsetX, offsetY, offsetZ, offsetRx, offsetRy, offsetRz)
        return self.sendRecvMsg(string)

    def SetCnvPointOffset(self, xOffset, yOffset):
        return self.sendRecvMsg("SetCnvPointOffset({:f},{:f})".format(xOffset, yOffset))

    def SetCnvTimeCompensation(self, time):
        return self.sendRecvMsg("SetCnvTimeCompensation({:d})".format(time))

    def StartSyncCnv(self):
        return self.sendRecvMsg("StartSyncCnv()")

    def StopSyncCnv(self):
        return self.sendRecvMsg("StopSyncCnv()")

    def TcpSendAndParse(self, cmd):
        """
        TcpSendAndParse command
        """
        return self.sendRecvMsg("TcpSendAndParse(\"{:s}\")".format(cmd))

    def Sleep(self, count):
        return self.sendRecvMsg("Sleep({:d})".format(count))

    def RelPointWeldLine(self, StartX, EndX, Y, Z, WorkAngle, TravelAngle, P1, P2):
        string = "RelPointWeldLine({:f},{:f},{:f},{:f},{:f},{:f},{{{:f},{:f},{:f},{:f},{:f},{:f}}},{{{:f},{:f},{:f},{:f},{:f},{:f}}})".format(
            StartX, EndX, Y, Z, WorkAngle, TravelAngle, P1[0], P1[1], P1[2], P1[3], P1[4], P1[5], P2[0], P2[1], P2[2], P2[3], P2[4], P2[5])
        return self.sendRecvMsg(string)

    def RelPointWeldArc(self, StartX, EndX, Y, Z, WorkAngle, TravelAngle, P1, P2, P3):
        string = "RelPointWeldArc({:f},{:f},{:f},{:f},{:f},{:f},{{{:f},{:f},{:f},{:f},{:f},{:f}}},{{{:f},{:f},{:f},{:f},{:f},{:f}}},{{{:f},{:f},{:f},{:f},{:f},{:f}}})".format(
            StartX, EndX, Y, Z, WorkAngle, TravelAngle, P1[0], P1[1], P1[2], P1[3], P1[4], P1[5], P2[0], P2[1], P2[2], P2[3], P2[4], P2[5], P3[0], P3[1], P3[2], P3[3], P3[4], P3[5])
        return self.sendRecvMsg(string)

    def WeaveStart(self):
        return self.sendRecvMsg("WeaveStart()")

    def WeaveParams(self, weldType, frequency, leftAmplitude, rightAmplitude, direction, stopMode, stopTime1, stopTime2, stopTime3, stopTime4, radius, radian, **kwargs):
        string = "WeaveParams({:d},{:f},{:f},{:f},{:d},{:d},{:d},{:d},{:d},{:d},{:f},{:f}".format(
            weldType, frequency, leftAmplitude, rightAmplitude, direction, stopMode, stopTime1, stopTime2, stopTime3, stopTime4, radius, radian)
        if kwargs:
            for key, value in kwargs.items():
                string += ",{}={}".format(key, value)
        string += ")"
        return self.sendRecvMsg(string)

    def WeaveEnd(self):
        return self.sendRecvMsg("WeaveEnd()")

    def WeldArcSpeedStart(self):
        return self.sendRecvMsg("WeldArcSpeedStart()")

    def WeldArcSpeed(self, speed):
        return self.sendRecvMsg("WeldArcSpeed({:f})".format(speed))

    def WeldArcSpeedEnd(self):
        return self.sendRecvMsg("WeldArcSpeedEnd()")

    def WeldWeaveStart(self, weldType, frequency, leftAmplitude, rightAmplitude, direction, stopMode, stopTime1, stopTime2, stopTime3, stopTime4, radius, radian):
        string = "WeldWeaveStart({:d},{:f},{:f},{:f},{:d},{:d},{:d},{:d},{:d},{:d},{:f},{:f})".format(
            weldType, frequency, leftAmplitude, rightAmplitude, direction, stopMode, stopTime1, stopTime2, stopTime3, stopTime4, radius, radian)
        return self.sendRecvMsg(string)


# Feedback interface
# 反饋資料介面類別


class DobotApiFeedBack(DobotApi):
    def __init__(self, ip, port, *args):
        super().__init__(ip, port, *args)
        self.__MyType = []
        self.last_recv_time = time.perf_counter()
        

    def feedBackData(self):
        """
        返回机械臂状态
        Return the robot status
        """
        self.socket_dobot.setblocking(True)  # 设置为阻塞模式
        data = bytes()
        current_recv_time = time.perf_counter() #计时，获取当前时间
        temp = self.socket_dobot.recv(144000) #缓冲区
        if len(temp) > 1440:    
            temp = self.socket_dobot.recv(144000)
        #print("get:",len(temp))
        i=0
        if len(temp) < 1440:
            while i < 5 :
                #print("重新接收")
                temp = self.socket_dobot.recv(144000)
                if len(temp) > 1440:
                    break
                i+=1
            if i >= 5:
                raise Exception("接收数据包缺失，请检查网络环境")
        
        interval = (current_recv_time - self.last_recv_time) * 1000  # 转换为毫秒
        self.last_recv_time = current_recv_time
        #print(f"Time interval since last receive: {interval:.3f} ms")
        
        data = temp[0:1440] #截取1440字节
        #print(len(data))
        #print(f"Single element size of MyType: {MyType.itemsize} bytes")
        self.__MyType = None   

        if len(data) == 1440:        
            self.__MyType = np.frombuffer(data, dtype=MyType)

        return self.__MyType
        
