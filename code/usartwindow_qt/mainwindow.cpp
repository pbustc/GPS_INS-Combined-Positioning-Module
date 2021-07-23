#include "mainwindow.h"
#include "ui_mainwindow.h"
#include<QtWebEngineWidgets>




MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setWindowTitle("GPS/BD&IMU"); //窗口名称
    //serialRead = false;  //对头文件中的一些变量进行初始化
    serialTimer = new QTimer(this);
    //初始化地图
    //此处使用绝对路径
    QString strMapPath = "D:/Qt/qt/usartwindow_new_test/map_test.html";
    QUrl url(strMapPath);
    ui->widget_map->load(url);
    PushButtonConnect();
}

MainWindow::~MainWindow()
{
    qsp->clear();
    qsp->close();
    delete ui;
}




void MainWindow::PushButtonConnect()
{
    //lambda函数写法，槽函数的内容比较简单的话，没必要再去定义一个槽来连接
    //pb_start为开始连接按钮，信号函数为当该按钮被点击
    //connect将信号函数与槽函数相连接，当信号函数发出信号时，执行如下指令
    initPort();
    connect(ui->pb_start,&QPushButton::clicked,this,[this](){

        //转变该按钮的文字，开始->断开或者断开->开始
        //ui中的pb_start为按钮的名字，而头文件函数中声明的pb_start为一个返回值为bool型的函数
        bool status=false;
        if(ui->pb_start->text()=="CONNECT")
            //status用于记录串口打开的状态，当连接失败时返回false，连接成功时返回true
            status=pb_start();
        else
            //当需要断开端口时
            pb_stop();
        //当status为true时，表示端口连接成功，此时按钮上显示断开连接
        status?ui->pb_start->setText("DISCONNECT"):ui->pb_start->setText("CONNECT");

        });
    //定时刷新串口
    //connect(serialTimer,&QTimer::timeout,this,&MainWindow::updatePort);
    //serialTimer->start(1000);  // 500ms超时
    connect(ui->pb_clear,&QPushButton::clicked,this,&MainWindow::pb_clear);
    //改变界面主题的按钮
    connect(ui->theme,&QPushButton::clicked,this,&MainWindow::theme);
    //地图加载完成后
    connect(ui->widget_map,SIGNAL(loadFinished(bool)),this,SLOT(MainWindow::afterLoadfinished(bool)));

    /*
    double lo = 107.733534;
    double la = 31.359755;
    QString command = QString ("addpoint(%0,%1)").arg(QString::number(lo,'f',6)).arg(QString::number(la,'f',6));
    ui->widget_map->page()->runJavaScript(command);
    */

}

/*端口的初始化*/
void MainWindow::initPort()
{
    //检查可用的端口，添加到下拉框以供选择
    foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts())
    {
        QSerialPort serial_num;
        serial_num.setPort(info);
        if(serial_num.open(QIODevice::ReadWrite))
        {
            ui->combox->addItem(serial_num.portName());
            serial_num.close();
        }
    }
}


/*串口的连接*/
bool MainWindow::pb_start()
{
    //头文件中声明了一个

    //获取可用串口
    QList<QSerialPortInfo> port = qspi->availablePorts();
    //选择连接串口
    for (int i=0;i<port.length();i++) {
        //currentText，当前项的文字
        if(port.at(i).portName()==ui->combox->currentText())
        {
            //串口号设置
            qsp->setPort(port.at(i));
            //串口波特率设置
            qsp->setBaudRate(ui->baudbox->currentText().toInt());
            //串口数据位设置
            qsp->setDataBits(QSerialPort::DataBits(ui->databox->currentText().toInt()));
            //串口停止位设置
            qsp->setStopBits(QSerialPort::StopBits(ui->stopbox->currentIndex()+1));
            //串口校验位设置
            qsp->setParity( (QSerialPort::Parity)(ui->checkbox->currentIndex()<1 ? \
                                                      ui->checkbox->currentIndex() : \
                                                      ui->checkbox->currentIndex()+1) );
            //开启串口
            qsp->open(QIODevice::ReadWrite);
            //修改按钮颜色
            ui->pb_start->setStyleSheet("background-color: #607D8B; color: white;");
            //监听串口 读取数据
            //readyRead--每当串口接收到信号便发出信号
            connect(qsp,&QSerialPort::readyRead,this,&MainWindow::read_port);
            return true;
        }
    }
    //串口连接失败时的函数处理
    QPoint point(this->x()+this->frameGeometry().width()/3,this->y()+this->frameGeometry().height()/3);
    QToolTip::showText(point,"CONNECT FAILURE");
//    QString cmd1 = "addmarker()";
//    ui->widget_map->page()->runJavaScript(cmd1);		// 直接page()下就可以运行
    connect(serialTimer,&QTimer::timeout,this,&MainWindow::map_update);
    serialTimer->start(1500);  // 500ms超时
    return false;
}
/*
 * 定时更新地图标点
*/
void MainWindow::map_update()
{
    QString command = QString ("addpoint(%1,%2)").arg(gps_lon).arg(gps_lat);
    ui->widget_map->page()->runJavaScript(command);
}
void MainWindow::pb_stop()
{
    //串口关闭
    qsp->close();
    //修改按钮颜色
    ui->pb_start->setStyleSheet("background-color: #E1E1E1; color: black;");
}

void MainWindow::pb_send()
{
    /*
    //判断发送类型
    if(ui->hexsend->isChecked()){
        //16进制转文本发送
        QString msg = ui->textEdit->toPlainText();
        QByteArray arr;
        bool ok = HexToString(msg,arr);
        if(ok){
            qsp->write(arr);
        }
    }else{
        //直接发送文本
        QString msg = ui->textEdit->toPlainText();
        qsp->write(msg.toUtf8());
    }
    */
}

void MainWindow::read_port()
{
    //读取收到的串口数据
    //readAll函数读取缓冲区当前已经接收的所有数据，若要求提取信号或格式化输出，则需要在keil中调整printf的值
    //在qt中接收到一定长度的值再进行readAll

    //QByteArray msg = qsp->readAll();
    //检查帧头和帧尾以提取数据
    QString rxString;
    QByteArray msg = qsp->readAll();
    //rxArray.append(msg);
    if( msg.length()!=0)
    {
        //从gps模块获取信息的次数

        gpsDatasProcessing(msg);
        //rxArray.clear();
        if(times % 10 == 0)
        {
            //ui->textedit_gps->clear();
            //ui->textedit_imu->clear();
        }
        //ui->textBrowser->append(tr("----------------------------------------------------------"));
    }
    else
    {
        //ui->textBrowser->append("未接收到信号");
        return;
    }

    /*if(msg.length() != 0)
    {
        //实现自动显示到最新内容，textBrower即为接收窗口的名称
        //moveCursor可移动窗口中的光标，此处为移动到末端
        ui->textBrowser->moveCursor(QTextCursor::End);
        //判断输出类型（16进制 or 文本）
        //isChecked可判断界面中的16进制输出是否被选中
        if(ui->hexdisplay->isChecked()){
            //当16进制输出被选中时，.tohex()函数中的参数为分隔符
            ui->textBrowser->append(msg.toHex(' '));
        }else{
            //append函数回自动换行
            //insertPlainText则插入什么就显示什么
            //
            ui->textBrowser->insertPlainText(QString::fromLocal8Bit(msg));
        }
    }*/
}

//接收数据的处理
void MainWindow::gpsDatasProcessing(QByteArray &GPSBuffer)
{
    /*QString GPSBufferString = QString(GPSBuffer);
    QList<QString> gpsStringList = GPSBufferString.split('\n');
    if(gpsStringList.at(0).at(0) != '$')
    {
        //数据发生黏合，纠正数据
        //将第一个qlist删除
        gpsStringList.erase(0);
    }
    if(gpsStringList.at(gpsStringList.length()-1).length()<15)
    {
        //最后一列数据没有完全接收，将其删除
        gpsStringList.erase(gpsStringList.end());
    }*/
    /*数据接收未完全*/
    if(GPSBuffer.at(GPSBuffer.length()-1) == '#')
    {
        rxArray.append(GPSBuffer);

        //按照每一行的数据进行处理,数据格式如下
        /*
        $info,beiJingTime.year+1900, beiJingTime.mon,beiJingTime.day,
        beiJingTime.hour,beiJingTime.min,beiJingTime.sec,deg_lat,deg_lon,info.elv,
        info.speed,info.satinfo.inuse,info.satinfo.inview,info.BDsatinfo.inuse,info.BDsatinfo.inview
        */
        QString rxString = QString(rxArray);
        QList<QString> TempList = rxString.split(',');
        /*
         year = TempList.at(1);
         mon = TempList.at(2);
         day = TempList.at(3);
         hour = TempList.at(4);
         min = TempList.at(5);
         sec = TempList.at(6);
         lat = TempList.at(7);
         lon = TempList.at(8);
         elv = TempList.at(9);
         speed = TempList.at(10);
         gpsSatellite_inuse = TempList.at(11);
         gpsSatellite_inview = TempList.at(12);
         BDSatellite_inuse = TempList.at(13);
         BDSatellite_inview = TempList.at(14);
         */
         gps_lat = TempList.at(1);
         gps_lon = TempList.at(2);

         gyro_x = TempList.at(3);
         gyro_y = TempList.at(4);
         gyro_z = TempList.at(5);

         accl_x = TempList.at(6);
         accl_y = TempList.at(7);
         accl_z = TempList.at(8);

         times++;
         rxString = QString(rxArray);
         ui->textedit_gps->moveCursor(QTextCursor::End);
         //ui->textedit_gps->setTextColor(QColor("green"));

         ui->textedit_gps->append(tr("----------------------------------------------------------"));
         ui->textedit_gps->append(QString::fromLocal8Bit("从GPS/北斗传感器第（")+QString::number(times)
                                  +QString::fromLocal8Bit("）次接收数据"));
         ui->textedit_gps->append(tr("----------------------------------------------------------"));
         //ui->textedit_gps->append(QString(rxArray));
        /*ui->textedit_gps->append("BeiJing Time:"+year+"-"+mon+"-"+day+"   "+hour+"-"+min+"-"+sec);
        ui->textedit_gps->append("Latitude:"+lat+"   "+"Longitude:"+lon);
        ui->textedit_gps->append("Altitude:"+elv);
        ui->textedit_gps->append("Speed:"+speed);
        ui->textedit_gps->append("Satellite(GPS) in use:"+gpsSatellite_inuse);
        ui->textedit_gps->append("Satellite(GPS) in view:"+gpsSatellite_inview);
        ui->textedit_gps->append("Satellite(BD) in use:"+BDSatellite_inuse);
        ui->textedit_gps->append("Satellite(BD) in view:"+BDSatellite_inview);*/
        ui->textedit_gps->append("Latitude:"+gps_lat+"   "+"Longitude:"+gps_lon);
        ui->textedit_imu->setTextColor(QColor("green"));
        ui->textedit_imu->moveCursor(QTextCursor::End);
        ui->textedit_imu->append(tr("----------------------------------------------------------"));
        ui->textedit_imu->append(QString::fromLocal8Bit("从IMU第（")+QString::number(times)+QString::fromLocal8Bit("）次接收数据"));
        ui->textedit_imu->append(tr("----------------------------------------------------------"));
        ui->textedit_imu->append("\nGYRO_X:"+gyro_x+"GYRO_Y:"+gyro_y+"GYRO_Z:"+gyro_z);
        ui->textedit_imu->append("\nACCL_X:"+accl_x+"ACCL_Y:"+accl_y+"ACCL_Z:"+accl_z+"\n");

        ui->lineEdit_lat->setText(gps_lat);
        ui->lineEdit_lon->setText(gps_lon);

        ui->lineEdit_acclx->setText(accl_x);
        ui->lineEdit_accly->setText(accl_y);
        ui->lineEdit_acclz->setText(accl_z);

        ui->lineEdit_gyrox->setText(gyro_x);
        ui->lineEdit_gyroy->setText(gyro_y);
        ui->lineEdit_gyroz->setText(gyro_z);



        /*此处用于标点进行在地图上的定位*/
        /*
        QString command = QString ("addpoint(%1,%2)").arg(gps_lon).arg(gps_lat);
        ui->widget_map->page()->runJavaScript(command);*/
        rxArray.clear();
    }
    else
    {
        rxArray.append(GPSBuffer);
    }
}



void MainWindow::pb_clear()
{
    //清屏保留样式表
    ui->textedit_gps->clear();
    theme();
}
void MainWindow::theme()
{
    //修改样式表实现不同显示效果
    if(ui->theme->isChecked())
    {
        ui->textedit_gps->setStyleSheet("background-color: black; color: white;");
        ui->textedit_imu->setStyleSheet("background-color: black; color: white;");
    }else{
        ui->textedit_gps->setStyleSheet("background-color: white; color: black;");
        ui->textedit_imu->setStyleSheet("background-color: white; color: black;");
    }
}

//16进制发送
void MainWindow::hexsend()
{
    /*
    //修改内容实现不同显示
    if(ui->hexsend->isChecked())
    {
        //16进制显示
        QString msg = ui->textEdit->toPlainText();
        ui->textEdit->setText(msg.toUtf8().toHex(' '));
    }else{
        //文本显示
        QString msg = ui->textEdit->toPlainText();
        QByteArray arr;
        bool ok = HexToString(msg,arr);
        if(ok){
          ui->textEdit->setText(arr);
        }
    }
    */
}

//实现16进制转为字符串//
//返回值 bool类型 ：true转换成功  false转换失败并且弹出提示框
//形参 QString,QByteArray 类型：
//                  msg 需要转换的16进制字符串，如 "33 34 35"
//                  ret 返回转换结果 ,  如 "345"
bool MainWindow::HexToString(QString &msg,QByteArray &ret)
{
    QByteArray arr;
    bool ok;
    QStringList sl = msg.split(' ');
    foreach (QString s,sl){
        if(!s.isEmpty()){
            char c = s.toInt(&ok,16)&0xFF;
            if(ok){
                arr.append(c);
            }else{
                QMessageBox::warning(this,"错误",QString("无效的16进制\'%1\'").arg(s));
                ok = false;
                break;
            }
            ok = true;
        }
    }
    if(ok){
        ret = arr;
    }
    return ok;
}

/*
 * 地图未加载完成时无法完成交互
 * 因此需要afterLoadfinished函数等待地图的加载完成
 * 之后进行初始化
*/
void MainWindow::afterLoadfinished(bool &a)
{
    if(a == true)
    {
        double lon[4], lat[4];
                lon[0] = 121.50866; lon[1] = 121.50863; lon[2] = 121.50899; lon[3] = 121.50902;
                lat[0] = 31.28867; lat[1] = 31.28860; lat[2] = 31.28845; lat[3] = 31.28855;
                for (int i = 0; i < 4; i++)
                {
                    QString command = QString("addpoint(%1,%2)").arg(QString::number(lon[i], 'f', 6)).arg(QString::number(lat[i], 'f', 6));
                    ui->widget_map->page()->runJavaScript(command);
                }
    }
}
