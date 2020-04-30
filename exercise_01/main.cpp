#include <iostream>
#include <fstream>
#include <array>

#include "Eigen.h"

#include "VirtualSensor.h"

struct Vertex {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // position stored as 4 floats (4th component is supposed to be 1.0)
    Vector4f position;
    // color stored as 4 unsigned char
    Vector4uc color;
};

bool WriteMesh(Vertex *vertices, unsigned int width, unsigned int height, const std::string &filename) {
    float edgeThreshold = 0.01f; // 1cm

    // TODO 2: use the OFF file format to save the vertices grid (http://www.geomview.org/docs/html/OFF.html)
    // - have a look at the "off_sample.off" file to see how to store the vertices and triangles
    // - for debugging we recommend to first only write out the vertices (set the number of faces to zero)
    // - for simplicity write every vertex to file, even if it is not valid (position.x() == MINF) (note that all vertices in the off file have to be valid, thus, if a point is not valid write out a dummy point like (0,0,0))
    // - use a simple triangulation exploiting the grid structure (neighboring vertices build a triangle, two triangles per grid cell)
    // - you can use an arbitrary triangulation of the cells, but make sure that the triangles are consistently oriented
    // - only write triangles with valid vertices and an edge length smaller then edgeThreshold

    // TODO: Get number of vertices
    unsigned int nVertices = 0;



    // TODO: Determine number of valid faces
    unsigned nFaces = 0;

    // Write off file
    std::ofstream outFile(filename);
    if (!outFile.is_open()) return false;

    // write header
    outFile << "COFF" << std::endl;
    outFile << nVertices << " " << nFaces << " 0" << std::endl;

    // TODO: save vertices


    // TODO: save valid faces


    // close file
    outFile.close();

    return true;
}

int main() {
    // Make sure this path points to the data folder
    std::string filenameIn = "../../data/rgbd_dataset_freiburg1_xyz/";
    std::string filenameBaseOut = "mesh_";

    // load video
    std::cout << "Initialize virtual sensor..." << std::endl;
    VirtualSensor sensor;
    if (!sensor.Init(filenameIn)) {
        std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
        return -1;
    }

    // convert video to meshes
    while (sensor.ProcessNextFrame()) {
        // get ptr to the current depth frame
        // depth is stored in row major (get dimensions via sensor.GetDepthImageWidth() / GetDepthImageHeight())
        float *depthMap = sensor.GetDepth();

        // get ptr to the current color frame
        // color is stored as RGBX in row major (4 byte values per pixel, get dimensions via sensor.GetColorImageWidth() / GetColorImageHeight())
        unsigned int width = sensor.GetColorImageWidth();
        unsigned int height = sensor.GetColorImageHeight();

        BYTE *colorMap = sensor.GetColorRGBX();

        // get depth intrinsics
        Matrix3f depthIntrinsics = sensor.GetDepthIntrinsics();
        float fX = depthIntrinsics(0, 0);
        float fY = depthIntrinsics(1, 1);
        float cX = depthIntrinsics(0, 2);
        float cY = depthIntrinsics(1, 2);

        // compute inverse depth extrinsics
        Matrix4f depthExtrinsicsInv = sensor.GetDepthExtrinsics().inverse();

        Matrix4f trajectory = sensor.GetTrajectory();
        Matrix4f trajectoryInv = sensor.GetTrajectory().inverse();

        // TODO 1: back-projection
        // write result to the vertices array below, keep pixel ordering!
        // if the depth value at idx is invalid (MINF) write the following values to the vertices array
        // vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
        // vertices[idx].color = Vector4uc(0,0,0,0);
        // otherwise apply back-projection and transform the vertex to world space, use the corresponding color from the colormap
        Vertex *vertices = new Vertex[sensor.GetDepthImageWidth() * sensor.GetDepthImageHeight()];

        Eigen::Matrix3f intrinsic; // camera space to image coordinates (pixels)
        intrinsic << fX, 0, cX,
                0, fY, cY,
                0, 0, 1;

        std::cout << intrinsic << std::endl;
        std::cout << "-----------------------" << std::endl;

        auto &depthIntrinsicInv = intrinsic.inverse();
        std::cout << depthIntrinsicInv << std::endl;

        std::cout << "-----------------------" << std::endl;

        std::cout << sensor.GetDepthExtrinsics() << std::endl;

        std::cout << "-----------------------" << std::endl;

        std::cout << depthExtrinsicsInv << std::endl;


        for (int i = 0; i < width; ++i) {
            for (int j = 0; j < height; ++j) {
                unsigned int idx = i + width * j;

                auto depth = depthMap[idx];

                if (depth == MINF) {
                    vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
                    vertices[idx].color = Vector4uc(0, 0, 0, 0);
                    continue;
                }

                // vertex3D = [ (u, v) @ intrinsicInv * depth ] @ extrensicInv

                Eigen::Vector3f imagePoint;

                imagePoint << i, j, 1;


                //                [3,1]            [3,3]          [3,1]
                Eigen::Vector3f cameraLine = depthIntrinsicInv * imagePoint;
                Eigen::Vector4f cameraPoint; // scalar multiple (homogeneous)
                cameraPoint << depth * cameraLine, 1; // extend

                //          [4,1]       [4,4]           [4,1]
                auto worldPoint = depthExtrinsicsInv * cameraPoint;

                // color
                auto colorIdx = idx * 4;
                auto red = colorMap[colorIdx + 0];
                auto green = colorMap[colorIdx + 1];
                auto blue = colorMap[colorIdx + 2];
                auto x = colorMap[colorIdx + 3];


                vertices[idx].position = worldPoint;
                vertices[idx].color = Vector4uc(red, green, blue, 0);

            }
        }

        // write mesh file
        std::stringstream ss;
        ss << filenameBaseOut << sensor.GetCurrentFrameCnt() << ".off";
        if (!WriteMesh(vertices, sensor.GetDepthImageWidth(), sensor.GetDepthImageHeight(), ss.str())) {
            std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
            return -1;
        }

        // free mem
        delete[] vertices;
    }

    return 0;
}
