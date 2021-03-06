#-------------------------------------------------
#
# Project created by QtCreator 2014-10-09T19:30:27
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = iris-scan-handler-qt
TEMPLATE = app

SOURCES += main.cpp\
        mainwindow.cpp \
    project.cpp \
    viewhandler.cpp \
    settingwindow.cpp

HEADERS  += mainwindow.h \
    project.h \
    viewhandler.h \
    settingwindow.h \

FORMS    += mainwindow.ui \
    settingwindow.ui

RC_FILE = resources.rc

RESOURCES += \
    resources.qrc

OTHER_FILES += \
    resources.rc \
    Kinect.ico

#Dependencies
INCLUDEPATH+=   "../../dependencies"

SOURCES +=  ../../dependencies/grayscaleregistration.cpp \
            ../../dependencies/icp.cpp \
            ../../dependencies/registration.cpp \
            ../../dependencies/surface.cpp \
            ../../dependencies/utils.cpp

HEADERS +=  ../../dependencies/grayscaleregistration.h \
            ../../dependencies/icp.h \
            ../../dependencies/registration.h \
            ../../dependencies/surface.h \
            ../../dependencies/typedef.h \
            ../../dependencies/utils.h

#Additional Includes with PCL and OpenCV
INCLUDEPATH+=   "C:/Program Files (x86)/Boost/include"
INCLUDEPATH+=   "C:/Program Files (x86)/Eigen/include"
INCLUDEPATH+=   "C:/Program Files (x86)/flann/include"
INCLUDEPATH+=   "C:/Program Files (x86)/OpenNI/Include"
INCLUDEPATH+=   "C:/Program Files (x86)/PCL/include/pcl-1.7"
INCLUDEPATH+=   "C:/Program Files (x86)/qhull/include"
INCLUDEPATH+=   "C:/Program Files (x86)/VTK 5.8.0/include/vtk-5.8"
INCLUDEPATH+=   "D:/Libraries/opencv/build/include/opencv"
INCLUDEPATH+=   "D:/Libraries/opencv/build/include/opencv2"
INCLUDEPATH+=   "D:/Libraries/opencv/build/include"

LIBS+=          "C:/Program Files (x86)/Microsoft SDKs/Windows/v7.0A/Lib/OpenGL32.lib"
LIBS+=          "C:/Program Files (x86)/Microsoft SDKs/Windows/v7.0A/Lib/User32.lib"
LIBS+=          "C:/Program Files (x86)/Microsoft SDKs/Windows/v7.0A/Lib/Gdi32.lib"

CONFIG(debug,debug|release) {
LIBS+=          "C:/Program Files (x86)/PCL/lib/pcl_kdtree_debug.lib"
LIBS+=          "C:/Program Files (x86)/PCL/lib/pcl_surface_debug.lib"
LIBS+=          "C:/Program Files (x86)/PCL/lib/pcl_io_debug.lib"
LIBS+=          "C:/Program Files (x86)/PCL/lib/pcl_search_debug.lib"
LIBS+=          "C:/Program Files (x86)/PCL/lib/pcl_segmentation_debug.lib"
LIBS+=          "C:/Program Files (x86)/PCL/lib/pcl_features_debug.lib"
LIBS+=          "C:/Program Files (x86)/PCL/lib/pcl_filters_debug.lib"
LIBS+=          "C:/Program Files (x86)/PCL/lib/pcl_visualization_debug.lib"
LIBS+=          "C:/Program Files (x86)/PCL/lib/pcl_common_debug.lib"
LIBS+=          "C:/Program Files (x86)/flann/lib/flann_cpp_s-gd.lib"
LIBS+=          "C:/Program Files (x86)/Boost/lib/libboost_chrono-vc100-mt-gd-1_50.lib"
LIBS+=          "C:/Program Files (x86)/Boost/lib/libboost_system-vc100-mt-gd-1_50.lib"
LIBS+=          "C:/Program Files (x86)/Boost/lib/libboost_filesystem-vc100-mt-gd-1_50.lib"
LIBS+=          "C:/Program Files (x86)/Boost/lib/libboost_thread-vc100-mt-gd-1_50.lib"
LIBS+=          "C:/Program Files (x86)/Boost/lib/libboost_date_time-vc100-mt-gd-1_50.lib"
LIBS+=          "C:/Program Files (x86)/Boost/lib/libboost_iostreams-vc100-mt-gd-1_50.lib"
LIBS+=          "C:/Program Files (x86)/OpenNI/Lib/openNI.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkalglib-gd.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkCharts-gd.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkCommon-gd.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkDICOMParser-gd.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkexoIIc-gd.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkexpat-gd.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkFiltering-gd.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkfreetype-gd.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkftgl-gd.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkGenericFiltering-gd.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkGeovis-gd.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkGraphics-gd.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkhdf5-gd.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkHybrid-gd.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkImaging-gd.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkInfovis-gd.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkIO-gd.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkjpeg-gd.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtklibxml2-gd.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkmetaio-gd.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkNetCDF-gd.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkNetCDF_cxx-gd.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkpng-gd.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkproj4-gd.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkRendering-gd.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtksqlite-gd.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtksys-gd.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtktiff-gd.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkverdict-gd.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkViews-gd.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkVolumeRendering-gd.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkWidgets-gd.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkzlib-gd.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/QVTK-gd.lib"
LIBS+=          "D:/Libraries/opencv/build/x86/vc10/lib/opencv_calib3d248d.lib"
LIBS+=          "D:/Libraries/opencv/build/x86/vc10/lib/opencv_contrib248d.lib"
LIBS+=          "D:/Libraries/opencv/build/x86/vc10/lib/opencv_core248d.lib"
LIBS+=          "D:/Libraries/opencv/build/x86/vc10/lib/opencv_features2d248d.lib"
LIBS+=          "D:/Libraries/opencv/build/x86/vc10/lib/opencv_flann248d.lib"
LIBS+=          "D:/Libraries/opencv/build/x86/vc10/lib/opencv_gpu248d.lib"
LIBS+=          "D:/Libraries/opencv/build/x86/vc10/lib/opencv_highgui248d.lib"
LIBS+=          "D:/Libraries/opencv/build/x86/vc10/lib/opencv_imgproc248d.lib"
LIBS+=          "D:/Libraries/opencv/build/x86/vc10/lib/opencv_legacy248d.lib"
LIBS+=          "D:/Libraries/opencv/build/x86/vc10/lib/opencv_ml248d.lib"
LIBS+=          "D:/Libraries/opencv/build/x86/vc10/lib/opencv_nonfree248d.lib"
LIBS+=          "D:/Libraries/opencv/build/x86/vc10/lib/opencv_objdetect248d.lib"
LIBS+=          "D:/Libraries/opencv/build/x86/vc10/lib/opencv_ocl248d.lib"
LIBS+=          "D:/Libraries/opencv/build/x86/vc10/lib/opencv_photo248d.lib"
LIBS+=          "D:/Libraries/opencv/build/x86/vc10/lib/opencv_stitching248d.lib"
LIBS+=          "D:/Libraries/opencv/build/x86/vc10/lib/opencv_superres248d.lib"
LIBS+=          "D:/Libraries/opencv/build/x86/vc10/lib/opencv_ts248d.lib"
LIBS+=          "D:/Libraries/opencv/build/x86/vc10/lib/opencv_video248d.lib"
LIBS+=          "D:/Libraries/opencv/build/x86/vc10/lib/opencv_videostab248d.lib"
} else {
LIBS+=          "C:/Program Files (x86)/PCL/lib/pcl_kdtree_release.lib"
LIBS+=          "C:/Program Files (x86)/PCL/lib/pcl_surface_release.lib"
LIBS+=          "C:/Program Files (x86)/PCL/lib/pcl_io_release.lib"
LIBS+=          "C:/Program Files (x86)/PCL/lib/pcl_search_release.lib"
LIBS+=          "C:/Program Files (x86)/PCL/lib/pcl_segmentation_release.lib"
LIBS+=          "C:/Program Files (x86)/PCL/lib/pcl_features_release.lib"
LIBS+=          "C:/Program Files (x86)/PCL/lib/pcl_filters_release.lib"
LIBS+=          "C:/Program Files (x86)/PCL/lib/pcl_visualization_release.lib"
LIBS+=          "C:/Program Files (x86)/PCL/lib/pcl_common_release.lib"
LIBS+=          "C:/Program Files (x86)/flann/lib/flann_cpp_s.lib"
LIBS+=          "C:/Program Files (x86)/Boost/lib/libboost_chrono-vc100-mt-1_50.lib"
LIBS+=          "C:/Program Files (x86)/Boost/lib/libboost_system-vc100-mt-1_50.lib"
LIBS+=          "C:/Program Files (x86)/Boost/lib/libboost_filesystem-vc100-mt-1_50.lib"
LIBS+=          "C:/Program Files (x86)/Boost/lib/libboost_thread-vc100-mt-1_50.lib"
LIBS+=          "C:/Program Files (x86)/Boost/lib/libboost_date_time-vc100-mt-1_50.lib"
LIBS+=          "C:/Program Files (x86)/Boost/lib/libboost_iostreams-vc100-mt-1_50.lib"
LIBS+=          "C:/Program Files (x86)/OpenNI/Lib/openNI.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkalglib.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkCharts.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkCommon.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkDICOMParser.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkexoIIc.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkexpat.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkFiltering.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkfreetype.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkftgl.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkGenericFiltering.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkGeovis.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkGraphics.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkhdf5.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkHybrid.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkImaging.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkInfovis.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkIO.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkjpeg.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtklibxml2.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkmetaio.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkNetCDF.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkNetCDF_cxx.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkpng.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkproj4.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkRendering.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtksqlite.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtksys.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtktiff.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkverdict.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkViews.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkVolumeRendering.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkWidgets.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/vtkzlib.lib"
LIBS+=          "C:/Program Files (x86)/VTK 5.8.0/lib/vtk-5.8/QVTK.lib"
LIBS+=          "D:/Libraries/opencv/build/x86/vc10/lib/opencv_calib3d248.lib"
LIBS+=          "D:/Libraries/opencv/build/x86/vc10/lib/opencv_contrib248.lib"
LIBS+=          "D:/Libraries/opencv/build/x86/vc10/lib/opencv_core248.lib"
LIBS+=          "D:/Libraries/opencv/build/x86/vc10/lib/opencv_features2d248.lib"
LIBS+=          "D:/Libraries/opencv/build/x86/vc10/lib/opencv_flann248.lib"
LIBS+=          "D:/Libraries/opencv/build/x86/vc10/lib/opencv_gpu248.lib"
LIBS+=          "D:/Libraries/opencv/build/x86/vc10/lib/opencv_highgui248.lib"
LIBS+=          "D:/Libraries/opencv/build/x86/vc10/lib/opencv_imgproc248.lib"
LIBS+=          "D:/Libraries/opencv/build/x86/vc10/lib/opencv_legacy248.lib"
LIBS+=          "D:/Libraries/opencv/build/x86/vc10/lib/opencv_ml248.lib"
LIBS+=          "D:/Libraries/opencv/build/x86/vc10/lib/opencv_nonfree248.lib"
LIBS+=          "D:/Libraries/opencv/build/x86/vc10/lib/opencv_objdetect248.lib"
LIBS+=          "D:/Libraries/opencv/build/x86/vc10/lib/opencv_ocl248.lib"
LIBS+=          "D:/Libraries/opencv/build/x86/vc10/lib/opencv_photo248.lib"
LIBS+=          "D:/Libraries/opencv/build/x86/vc10/lib/opencv_stitching248.lib"
LIBS+=          "D:/Libraries/opencv/build/x86/vc10/lib/opencv_superres248.lib"
LIBS+=          "D:/Libraries/opencv/build/x86/vc10/lib/opencv_ts248.lib"
LIBS+=          "D:/Libraries/opencv/build/x86/vc10/lib/opencv_video248.lib"
LIBS+=          "D:/Libraries/opencv/build/x86/vc10/lib/opencv_videostab248.lib"
}
