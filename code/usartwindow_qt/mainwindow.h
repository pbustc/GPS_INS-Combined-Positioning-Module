#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include<QtWebEngineWidgets>
#include <QMainWindow>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QDebug>
#include <QMessageBox>
#include <QToolTip>
#include <QTimer>
QT_BEGIN_NAMESPACE

namespace Ui { class MainWindow; }
QT_END_NAMESPACE
using namespace std;
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    //实例化一个串口类
    //QSerialPort提供了操作串口的各种接口
    //QSerialPortInfo是一个辅助类，可以提供计算机中可用串口的各种信息

    QSerialPort *qsp = new QSerialPort();
    QSerialPortInfo *qspi = new QSerialPortInfo();
    int times = 0; //接收次数
    QByteArray rxArray; //创建字符数组
    QTimer *timer;  //定时触发器
    QTimer *serialTimer;
    bool serialRead; //是否读取串口，与slotSerialTimeout函数连接，每隔一段时间读取一次信息
    QString year,mon,day,hour,min,sec,lat,lon,elv,speed,gpsSatellite_inuse,gpsSatellite_inview,BDSatellite_inuse,BDSatellite_inview;
    QString gps_lon,gps_lat,gyro_x,gyro_y,gyro_z,accl_x,accl_y,accl_z;

    void PushButtonConnect();
public slots:
    bool pb_start();
    void pb_stop();
    void pb_send();
    void read_port();
    void gpsDatasProcessing(QByteArray &GPSBuffer);
    void pb_clear();
    void theme();
    void hexsend();
    void map_update();
    bool HexToString(QString &msg,QByteArray &arr);
    //void slotSerialTimerOut();
    void afterLoadfinished(bool &a);
    void initPort();
};
#endif // MAINWINDOW_H
