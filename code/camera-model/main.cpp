#include <iostream>
#include <Eigen/Dense> // must include Eigen before <h5cpp/core>
#include <h5cpp/all>


#include <QApplication>


#include <vtkImageViewer.h>
#include <vtkJPEGReader.h>
#include <vtkSmartPointer.h>
#include <vtkImageData.h>
#include <vtkNew.h>
#include <vtkActor2D.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkImageMapper.h>
#include <vtkOpenGLImageMapper.h>

#include "QVTKWidget.h"

Eigen::Matrix3d
load_calibration(const std::string& filepath) {
    auto fd = h5::open(filepath, H5F_ACC_RDONLY);
    Eigen::MatrixXd K = h5::read<Eigen::MatrixXd>(fd, "/NP1_ir_K").transpose();
    return K;
}

vtkSmartPointer<vtkImageData>
load_depth_image(const std::string& filepath) {
    auto fd = h5::open(filepath, H5F_ACC_RDONLY);
    Eigen::MatrixXi depth = h5::read<Eigen::MatrixXi>(fd, "/depth");
    std::cout << "r: " << depth.rows() << "; c: " << depth.cols() << "\n";
    std::cout << depth.block(0, 0, 10, 10) << "\n";

    vtkSmartPointer<vtkImageData> depthimagedata = vtkSmartPointer<vtkImageData>::New();
    depthimagedata->SetDimensions(depth.rows(), depth.cols(), 1);
    depthimagedata->AllocateScalars(VTK_UNSIGNED_CHAR, 1);
    for (int x = 0; x < depth.rows(); x++) {
        for (int y = 0; y < depth.cols(); y++) {
            for (int z = 0; z < 1; z++) {
                unsigned char* ptr = (unsigned char*) depthimagedata->GetScalarPointer(x, y, z);
                *ptr = depth(x, depth.cols() - y - 1);
            }
        }
    }
    return depthimagedata;
}

vtkSmartPointer<vtkImageData>
load_jpg_image(const std::string& filepath) {
    vtkNew<vtkJPEGReader> jpegReader;
    jpegReader->SetFileName(filepath.c_str());

    vtkSmartPointer<vtkImageData> jpegimagedata = jpegReader->GetOutput();
    jpegReader->Update();
    return jpegimagedata;
}

void show_image(int argc, char** argv, vtkSmartPointer<vtkImageData> imagedata) {
    QApplication app(argc, argv);

    QVTKWidget widget;


    vtkSmartPointer<vtkImageViewer> image_view = vtkSmartPointer<vtkImageViewer>::New();
    image_view->SetInputData(imagedata);
    int* dims = imagedata->GetDimensions();
    widget.resize(dims[0], dims[1]);

    widget.SetRenderWindow(image_view->GetRenderWindow());
    image_view->SetupInteractor(widget.GetRenderWindow()->GetInteractor());

    image_view->SetColorLevel(138.5);
    image_view->SetColorWindow(233);

    widget.show();

    app.exec();
}

int main(int argc, char** argv) {
    vtkSmartPointer<vtkImageData> depthimagedata = load_depth_image("./NP1_0.h5");
    vtkSmartPointer<vtkImageData> jpegimagedata = load_jpg_image("./NP1_0.jpg");
    Eigen::Matrix3d K = load_calibration("calibration.h5");
    std::cout << K << "\n";


    show_image(argc, argv, jpegimagedata);
    show_image(argc, argv, depthimagedata);

    return EXIT_SUCCESS;
}


