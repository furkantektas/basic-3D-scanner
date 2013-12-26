#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <iostream>
#include <vector>

#include <QObject>
#include <QThread>
#include <QRunnable>
#include <QTimer>
#include <QMainWindow>
#include <QString>
#include <QProgressDialog>

#include <boost/thread/mutex.hpp>

#include <pcl/io/openni_grabber.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>


#include "motordriver.h"
#include "PCDIO.h"
#include "PCLOperations.h"
#include "types.h"

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow, public QRunnable {
    Q_OBJECT
public:
    typedef pcl::visualization::PCLVisualizer Visualizer;
    typedef boost::shared_ptr<Visualizer> VisualizerPtr;
    static const int STEP_COUNT_PER_ROTATE = 50;
    
    explicit MainWindow(QWidget *parent = 0);
    virtual ~MainWindow();
    void cloudCallBack(const ColorCloudConstPtr& cloud);

    void meshToTriangle();
    void run();
    void updateClippingBox();

    inline bool isCapturing() { return _capture; }
    inline void capture() { _capture = !_capture; }
    inline int getProgress() { return _progress; }

    inline float getXMin () const {return (_x_min);}
    inline float getXMax () const {return (_x_max);}
    inline float getYMin () const {return (_y_min);}
    inline float getYMax () const {return (_y_max);}
    inline float getZMin () const {return (_z_min);}
    inline float getZMax () const {return (_z_max);}

    inline int getCaptureCount () const {return (_captureCount);}
public slots:
    void setXMin(double newXMin);
    void setXMax(double newXMax);
    void setYMin(double newYMin);
    void setYMax(double newYMax);
    void setZMin(double newZMin);
    void setZMax(double newZMax);
    void setCaptureCount (int count);
    void onCaptureClick();
    void onRefreshClick();
    void onProgressUpdate(int progress, const QString& statusMsg);
    void visualize();
    void showCloudMesh(bool status);
    //void setCaptureCount(int count);
signals:
    void updateKinectPixmap(QPixmap pixmap);
private:
    Ui::MainWindow *ui;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> _visualizer;
    
    ColorCloudConstPtr _latestCloud;
    ColorCloudConstPtr _combinedCloud;
    Mesh _combinedMesh;
    mutable boost::mutex _cloud_mutex;
    mutable boost::mutex _visualizer_mutex;
    std::vector<ColorCloudPtr> _clouds;
    pcl::Grabber* _grabber;
    MotorDriver _motorDriver;
    bool _showMesh;
    QTimer *_vis_timer;
    QTimer *_oneTimeEvent;
    bool running;
    bool _capture;
    int _progress;
    int _rotationCount;
    int _captureCount;
    
    // Clipping boundaries in centimeters
    float _x_min;
    float _x_max;
    float _y_min;
    float _y_max;
    float _z_min;
    float _z_max;
    QProgressDialog *_progressDialog;

    inline void setProgress(int progress) { _progress = progress; }
    void progressUpdate(int progress,const char* status);
    int getSleepTime();
    
    void refreshFormElements();
    void updateKinectStatusOff();
    void updateKinectStatusOn();
    void updateKinectStatusDisconnected();
private slots:
    void init();
};

#endif // MAINWINDOW_H
