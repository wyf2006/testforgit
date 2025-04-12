#include "vex.h"
 #include <cmath>

 using namespace vex;
 //brain Brain;
 
 //================ 设备初始化 ================
 // 蓝色电机 - 使用6:1齿轮比，连接至端口1
 motor BlueMotor = motor(PORT1, ratio6_1, false);
 
 // 红色电机 - 使用36:1齿轮比，连接至端口2
 motor RedMotor = motor(PORT2, ratio36_1, false);
 
 // 惯性传感器(陀螺仪) - 连接至端口3
 inertial Gyro = inertial(PORT3);
 
 // 距离传感器 - 连接至端口4
 distance DistanceSensor = distance(PORT4);
 
 // 光学传感器 - 连接至端口5
 optical opticalSensor = optical(PORT11);
 
 // 角度传感器 - 连接至端口6
 rotation AngleSensor = rotation(PORT6);
 
 // 控制器 - 无需端口连接
 controller Controller = controller();
 
 // 数字输出(气缸) 
 digital_out digout = digital_out(Brain.ThreeWirePort.A);


 
 //================ 状态变量 ================
 bool redMotorMoved = false;        // 红电机是否已移动（任务2）
 bool blueMotorMoved = false;       // 蓝电机是否已移动（任务3）
 bool blueMotorReverse = false;     // 蓝电机是否反转（任务3）
 bool blueMotorSpinSlow = false;    // 蓝电机是否慢速（任务3）
 bool enableTaskOne = true;         // 是否启用任务1
 bool enableTaskThree = true;       // 是否启用任务3
    //这里本来我不想用任务启用标志，直接靠互斥锁，但是claude声称这样可以节省系统开销，所以就留着吧
 bool task5Active = false;          // 任务5是否激活
 motor_group MotorGroup1 = motor_group(BlueMotor, RedMotor);
 bool pidActive = false;            // 任务6是否激活
 double targetAngle = 400;          // PID目标角度
 double kP = 0.5, kI = 0.1, kD = 0.05; // PID参数
 volatile bool maintainRedmotorDegree = false;
 int pidTimes=0;

  //================ 互斥锁 ================
 mutex BlueMotorMutex;  // 蓝电机互斥锁
 mutex RedMotorMutex;   // 红电机互斥锁
 mutex Task5Mutex;      // 任务5状态互斥锁
 
 //================ 任务1函数 ================
 // 蓝电机控制函数 - 任务1
 int controlBlueMotor() {
     while (true) {
         if (Controller.ButtonX.pressing() && enableTaskOne) {
             BlueMotorMutex.lock();
             BlueMotor.spin(forward, 100, rpm);
             BlueMotorMutex.unlock();
         }
         this_thread::sleep_for(20);
     }
     return 0;
 }
 
 // 蓝电机停止函数 - 任务1回调
 void BlueMotorStop() {
     if (enableTaskOne) {//这里是检测是否启用了这个任务1，没有就直接跳过不管
         BlueMotorMutex.lock();
         BlueMotor.stop();
         BlueMotorMutex.unlock();
     }
 }
 
 //================ 任务2函数 ================
 // 红电机控制函数 - 任务2回调
 void RedMotorSpin() {
     // 如果PID正在运行，忽略此命令
     if (pidActive) {
         return;
     }
     
     RedMotorMutex.lock();
     if (!redMotorMoved) {
         RedMotor.spinFor(600, degrees, 100, rpm, false);
         redMotorMoved = true;
     } else {
        // RedMotor.resetPosition();
        RedMotor.spinTo(0,degrees);
         redMotorMoved = false;
     }
     RedMotorMutex.unlock();
 }
 
 //================ 任务3函数 ================
 // 蓝电机高级控制 - 任务3
 int advancedBlueMotorControl() {
     while (true) {
         if (!enableTaskThree) {
             this_thread::sleep_for(20);
             continue;
         }
         
         // A按钮：开始/停止蓝电机
         if (Controller.ButtonA.pressing()) {
             BlueMotorMutex.lock();
             if (!blueMotorMoved) {
                 if (blueMotorReverse) {
                     BlueMotor.spin(reverse, blueMotorSpinSlow ? 100 : 600, rpm);
                 } else {
                     BlueMotor.spin(forward, blueMotorSpinSlow ? 100 : 600, rpm);
                 }
                 blueMotorMoved = true;
             } else {
                 BlueMotor.stop();
                 blueMotorMoved = false;
             }
             BlueMotorMutex.unlock();
             
             // 等待按钮释放
             while (Controller.ButtonA.pressing()) {
                 this_thread::sleep_for(20);
             }
         }
 
         // B按钮：切换蓝电机旋转方向
         if (Controller.ButtonB.pressing()) {
             blueMotorReverse = !blueMotorReverse;
             
             // 如果电机正在运行，实时更新方向
             if(blueMotorMoved) {
                 BlueMotorMutex.lock();
                 if(blueMotorReverse) {
                     BlueMotor.spin(reverse, blueMotorSpinSlow ? 100 : 600, rpm);
                 } else {
                     BlueMotor.spin(forward, blueMotorSpinSlow ? 100 : 600, rpm);
                 }
                 BlueMotorMutex.unlock();
             }
             
             // 等待按钮释放
             while (Controller.ButtonB.pressing()) {
                 this_thread::sleep_for(20);
             }
         }
 
         // Y按钮：切换蓝电机速度
         if (Controller.ButtonY.pressing()) {
             blueMotorSpinSlow = !blueMotorSpinSlow;
             
             // 如果电机正在运行，实时更新速度
             if(blueMotorMoved) {
                 BlueMotorMutex.lock();
                 if(blueMotorReverse) {
                     BlueMotor.spin(reverse, blueMotorSpinSlow ? 100 : 600, rpm);
                 } else {
                     BlueMotor.spin(forward, blueMotorSpinSlow ? 100 : 600, rpm);
                 }
                 BlueMotorMutex.unlock();
             }
             
             // 等待按钮释放
             while (Controller.ButtonY.pressing()) {
                 this_thread::sleep_for(20);
             }
         }
 
         this_thread::sleep_for(20);
     }
     return 0;
 }
 
 //================ 任务4函数 ================
 // 陀螺仪数据打印函数 - 任务4
 int printGyroData() {
     while (true) {
         // 读取陀螺仪数据
         double rotation = Gyro.rotation();  // 获取陀螺仪的累计旋转角度
         double heading = Gyro.heading();    // 获取航向角，类似于指南针方向
         double yaw = Gyro.yaw();            // 获取偏航角，测量机器人围绕垂直轴的旋转（左右转向），与heading类似但可为负值
         double pitch = Gyro.pitch();        // 获取俯仰角，测量机器人围绕横轴的倾斜度（前后倾斜），正值表示前倾，负值表示后倾
         double roll = Gyro.roll();          // 获取横滚角，测量机器人围绕纵轴的倾斜度（左右倾斜），正值表示右倾，负值表示左倾
         
         // 读取角度传感器
         double angle = AngleSensor.position(degrees);
         
         // 打印到主屏幕
         Brain.Screen.clearScreen();
         Brain.Screen.setCursor(1, 1);
         Brain.Screen.print("Rotation: %.2f", rotation);
         Brain.Screen.setCursor(2, 1);
         Brain.Screen.print("Heading: %.2f", heading);
         Brain.Screen.setCursor(3, 1);
         Brain.Screen.print("Yaw: %.2f", yaw);
         Brain.Screen.setCursor(4, 1);
         Brain.Screen.print("Pitch: %.2f", pitch);
         Brain.Screen.setCursor(5, 1);
         Brain.Screen.print("Roll: %.2f", roll);
         
         // 添加传感器数据（既然有那也打上去吧，判断蓝环的时候还挺好用的）
         double distance = DistanceSensor.objectDistance(mm);
         double hue = opticalSensor.hue();
         
         Brain.Screen.setCursor(6, 1);
         Brain.Screen.print("Distance: %.1f mm", distance);
         Brain.Screen.setCursor(7, 1);
         Brain.Screen.print("Hue: %.1f", hue);
         Brain.Screen.setCursor(8, 1);
         Brain.Screen.print("Angle: %.1f deg", angle);
         Brain.Screen.setCursor(9, 1);
         Brain.Screen.print("PidTimes: %ld", pidTimes);
         Brain.Screen.setCursor(10, 1);
         Brain.Screen.print("User: wyf2006");//宣誓主权
         
         // 任务状态
         Task5Mutex.lock();
         bool t5status = task5Active;
         Task5Mutex.unlock();
         
         Brain.Screen.setCursor(12, 1);
         //Brain.Screen.print("Task 5: %s", t5status ? "Active" : "Ready");
         Brain.Screen.setCursor(13, 1);
         //Brain.Screen.print("Task 6: %s", pidActive ? "Active" : "Ready");
         
         
         
         this_thread::sleep_for(200);
     }
     return 0;
 }
 
 //================ 任务5函数 ================
 // 任务5按钮回调（R2按钮按下时执行）
 void task5Trigger() {
     // 检查任务5是否已经激活
     Task5Mutex.lock();
     bool isActive = task5Active;
     Task5Mutex.unlock();
     
     // 如果任务5已经激活，则忽略此次触发
     if (isActive) {
         return;
     }
     
     // 标记任务5为激活状态
     Task5Mutex.lock();
     task5Active = true;
     Task5Mutex.unlock();
     
     // 禁用其他任务
     enableTaskOne = false;
     enableTaskThree = false;
     
     // 第1步：启动电机 - 正向
     BlueMotorMutex.lock();
     RedMotorMutex.lock();
     MotorGroup1.spin(forward, 400, rpm);
     BlueMotorMutex.unlock();
     RedMotorMutex.unlock();
     
     // 第2步：运行任务逻辑
     timer taskTimer;
     taskTimer.reset();
     bool taskDone = false;
     
     while(taskTimer.time() < 10000 && !taskDone) {
         // 检测蓝色物体
         if ((opticalSensor.hue() >= 200)&&(opticalSensor.hue() <= 220)) {
             // 停止电机
             BlueMotorMutex.lock();
             RedMotorMutex.lock();
             MotorGroup1.stop();
             BlueMotorMutex.unlock();
             RedMotorMutex.unlock();
            // 打开气缸 (已注释掉)
             //digout.set(true);
             
             // 等待1秒
             wait(1, sec);
             
             taskDone = true;
         }
         
         // 检测障碍物
         if (DistanceSensor.objectDistance(mm) < 100) {
             // 反向运动
             BlueMotorMutex.lock();
             RedMotorMutex.lock();
             MotorGroup1.spin(reverse, 400, rpm);
             BlueMotorMutex.unlock();
             RedMotorMutex.unlock();
             // 等待2秒
             wait(2, sec);
             
             taskDone = true;
         }
         
         // 短暂等待再次检测
         wait(20, msec);
     }
    // 第3步：关闭气缸 (已注释掉)
     
     // 第4步：停止所有电机
    BlueMotorMutex.lock();
    RedMotorMutex.lock();
    MotorGroup1.stop();
    BlueMotorMutex.unlock();
    RedMotorMutex.unlock();
     
     // 第6步：恢复其他任务
    enableTaskOne = true;
    enableTaskThree = true;
     
     // 第7步：标记任务5为未激活状态
    Task5Mutex.lock();
    task5Active = false;
    Task5Mutex.unlock();
 }
//================ 任务6函数 ================
void pidControl() {
    // 重置角度传感器
    AngleSensor.resetPosition();
    
    // 简单PID参数
    double targetAngle = 400;  // 目标角度
    double kP = 0.2;            // 大幅降低比例系数
    double kI = 0.0003;            // 暂时完全禁用积分项
    double kD = 0.05;           // 大幅增加微分系数
    // PID变量
    double error = 0;
    double integral = 0;
    double previousError = 0;
    
    
    // 控制循环
    while (maintainRedmotorDegree) {
        // 读取当前角度
        double currentAngle = AngleSensor.position(degrees);
        
        // 计算误差
        error = targetAngle - currentAngle;
        
        // 积分项 (简单累加)
        integral += error;
        
        // 微分项
        double derivative = error - previousError;
        
        // 计算输出
        double motorSpeed = kP * error+kI* integral+ kP*derivative ;
        
        // 限制输出范围
        if (motorSpeed > 100) motorSpeed = 100;
        if (motorSpeed < -100) motorSpeed = -100;
        
        // 控制电机
        RedMotorMutex.lock();
        RedMotor.spin(reverse, motorSpeed, rpm);
        RedMotorMutex.unlock();
        
        // 更新前一个误差
        previousError = error;
        this_thread::sleep_for(20); 
        pidTimes++;
        
        
    }
    
    // 停止电机
    RedMotorMutex.lock();
    RedMotor.stop();
    RedMotorMutex.unlock();
}
// 停止 PID 控制
void stopPidControl() {
    maintainRedmotorDegree = false;
}
 
 //================ 主函数 ================
 int main() {
     // 初始化 VEX 设备配置
     vexcodeInit();
   
     // 重置电机和传感器
     BlueMotor.resetPosition();
     RedMotor.resetPosition();
     AngleSensor.resetPosition();
   
     // 初始化陀螺仪并等待校准完成
     Gyro.calibrate();
     wait(200, msec);
     
     // 初始化光学传感器
     opticalSensor.setLight(ledState::on);
   
     // 设置按钮回调
     Controller.ButtonX.released(BlueMotorStop);
     Controller.ButtonR1.pressed(RedMotorSpin);
     Controller.ButtonR2.pressed(task5Trigger);
   
     // 启动任务线程1-4
     thread t1(controlBlueMotor);
     thread t3(advancedBlueMotorControl);
     thread t4(printGyroData);
     
     // 让线程分离，独立运行
     t1.detach();
     t3.detach();
     t4.detach();
 
     // 初始化PID变量
     double previousError = 0, integral = 0;
     timer pidTimer;
 
     // 主循环保持程序运行
     while(1) {
         // 显示状态信息在屏幕底部
         Brain.Screen.setCursor(14, 1);
         Brain.Screen.print("Tasks 1-6 Ready");
         Brain.Screen.setCursor(15, 1);
         Brain.Screen.print("R2: Task5 | L/R: Task6");
         
         // 如果左按钮被按下，启动PID控制 (任务6)
      if (Controller.ButtonLeft.pressing()) {
        maintainRedmotorDegree = true;  // 设置标志位，维持红色电机角度
        // 创建并启动 PID 控制的线程
        thread pidThread(pidControl);
        pidThread.detach();  // 将线程与主线程分离，允许独立运行
        // 等待直到左按钮被释放
        while (Controller.ButtonLeft.pressing()) {
            this_thread::sleep_for(20);  // 休眠20毫秒，防止高频循环占用CPU
        }
    }

    // 如果右按钮被按下，停止PID控制 (任务6)
    if (Controller.ButtonRight.pressing()) {
        stopPidControl();  // 调用函数停止 PID 控制
        // 等待直到右按钮被释放
        while (Controller.ButtonRight.pressing()) {
            this_thread::sleep_for(20);  // 休眠20毫秒，防止高频循环占用CPU
        }
    }
  this_thread::sleep_for(20);  // 休眠20毫秒，减轻CPU负担

     }
 }