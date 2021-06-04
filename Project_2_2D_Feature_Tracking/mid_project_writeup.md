# Midterm Project WriteUp

## MP.1 Data Buffer Optimization

Implement a vector for dataBuffer objects whose size does not exceed a limit (e.g. 2 elements). This can be achieved by pushing in new elements on one end and removing elements on the other end.

Approach: `std::vector::erase` is used to erase the first element (the oldest image) in the vector. The incoming image is pushed to the back of the vector.

Let dataBufferSize = 2,

```cs
if (dataBuffer.size() > dataBufferSize)
{
    dataBuffer.erase(dataBuffer.begin());
    dataBuffer.push_back(frame);
}
else
{
    dataBuffer.push_back(frame);
}
```

## MP.2 Keypoint Detection

Implement detectors HARRIS, FAST, BRISK, ORB, AKAZE, and SIFT and make them selectable by setting a string accordingly.

Approach: string `detectorType` is used to select different type of detector. Then, `detectorType` is passed to the respective function (`detKeypointsShiTomasi` or `detKeypointsHarris` or `detKeypointsModern`). Harris detector can be implemented by measuring corner response and performing non-maxima suppression. FAST, BRISK, ORB, AKAZE, and SIFT detectors is available in OpenCV.

```cs
// matching2D_Student.cpp

vector<cv::KeyPoint> keypoints; // create empty feature list for current image
string detectorType = "SHITOMASI";

if (detectorType.compare("SHITOMASI") == 0)
{
    detectTime = detKeypointsShiTomasi(keypoints, imgGray, true);
}
else if (detectorType.compare("HARRIS") == 0)
{
    detectTime = detKeypointsHarris(keypoints, imgGray, true);
}
else
{
    detectTime = detKeypointsModern(keypoints, imgGray, detectorType, true);
}
```

```cs
// matching2D_Student.cpp

// HARRIS detector
double detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // Detector parameters
    int blockSize = 2;     // for every pixel, a blockSize × blockSize neighborhood is considered
    int apertureSize = 3;  // aperture parameter for Sobel operator (must be odd)
    int minResponse = 100; // minimum value for a corner in the 8bit scaled response matrix
    double k = 0.04;       // Harris parameter (see equation for details)

    // Detect Harris corners and normalize output
    double t = (double)cv::getTickCount();
    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros(img.size(), CV_32FC1);
    cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
    cv::convertScaleAbs(dst_norm, dst_norm_scaled);

    float maxOverlap = 0.0f;
    for (int i = 0; i < dst.rows; i++)
    {
        for (int j = 0; j < dst.cols; j++)
        {
            int currentMinResponse = (int)dst_norm.at<float>(i,j);
            bool bOverlap = false;
            if (currentMinResponse > minResponse)
            {
                cv::KeyPoint newKeypoint;
                newKeypoint.pt = cv::Point2f(j, i);
                newKeypoint.response = currentMinResponse;
                newKeypoint.size = 2 * apertureSize;

                for (auto it = keypoints.begin(); it != keypoints.end(); ++it)
                {
                    float result = cv::KeyPoint::overlap(newKeypoint, *it);
                    if (result > maxOverlap)
                    {
                        bOverlap = true;
                        if (currentMinResponse > (*it).response)
                        {
                            (*it) = newKeypoint;
                            break;
                        }
                    }
                }
                if (!bOverlap)
                {
                    keypoints.push_back(newKeypoint);
                }
            }
        }
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    double detectTime = 1000 * t / 1.0;
    return detectTime;
}
```

```cs
// matching2D_Student.cpp

// FAST, BRISK, ORB, AKAZE, and SIFT detectors
double detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType, bool bVis)
{
    double t = (double)cv::getTickCount();
    if (detectorType.compare("FAST") == 0)
    {
        int threshold = 30;  // difference between intensity of the central pixel and pixels of a circle around this pixel
        bool bNMS = true;    // perform non-maxima suppression on keypoints
        cv::FastFeatureDetector::DetectorType type = cv::FastFeatureDetector::TYPE_9_16; // TYPE_9_16, TYPE_7_12, TYPE_5_8
        cv::Ptr<cv::FeatureDetector> detector = cv::FastFeatureDetector::create(threshold, bNMS, type);
        detector->detect(img, keypoints);
    }
    else if (detectorType.compare("ORB") == 0)
    {
        cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
        detector->detect(img, keypoints);
    }
    else if (detectorType.compare("BRISK") == 0)
    {
        cv::Ptr<cv::FeatureDetector> detector = cv::BRISK::create();
        detector->detect(img, keypoints);
    }
    else if (detectorType.compare("AKAZE") == 0)
    {
        cv::Ptr<cv::FeatureDetector> detector = cv::AKAZE::create();
        detector->detect(img, keypoints);
    }
    else if (detectorType.compare("SIFT") == 0)
    {
        cv::Ptr<cv::FeatureDetector> detector = cv::SIFT::create();
        detector->detect(img, keypoints);
    }
    else
    {
        std::cout << "no detector type is found." << "\n";
        exit(1);
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    double detectTime = 1000 * t / 1.0;
    return detectTime;
}
```

## MP.3 Keypoint Removal

Remove all keypoints outside of a pre-defined rectangle and only use the keypoints within the rectangle for further processing.

Approach: First we define the boundary of the rectangle that contains the preceding vehicle. If the position of a keypoint is out of rectangle, that keypoint will be discarded.

```cs
// MidTermProject_Camera_Student.cpp

bool bFocusOnVehicle = true;
cv::Rect vehicleRect(535, 180, 180, 150);
if (bFocusOnVehicle)
{
    vector<cv::KeyPoint> carKeyPoints;
    for (auto it = keypoints.begin(); it != keypoints.end(); ++it)
    {
        float x = (*it).pt.x;
        float y = (*it).pt.y;
        if (x >= vehicleRect.x && x <= vehicleRect.x + vehicleRect.width &&
            y >= vehicleRect.y && y <= vehicleRect.y + vehicleRect.height)
        {
            carKeyPoints.push_back(*it);
        }
    }
    keypoints = carKeyPoints;
}
```

## MP.4 Keypoint Descriptors

Implement descriptors BRIEF, ORB, FREAK, AKAZE and SIFT and make them selectable by setting a string accordingly.

Approach: string `descriptorType` is used to switch descriptor type. Then, `descriptorType` is passed to the function `descKeypoints`. The extractor of each descriptor type is available in OpenCV.

```cs
// MidTermProject_Camera_Student.cpp

cv::Mat descriptors;
string descriptorType = "BRISK"; // BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT
double extractTime = descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);
```

```cs
// matching2D_Student.cpp

double descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)
{
    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType.compare("BRISK") == 0)
    {
        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    else if (descriptorType.compare("BRIEF") == 0) // BRIEF, ORB, FREAK, AKAZE, SIFT
    {
        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
    }
    else if (descriptorType.compare("ORB") == 0)
    {
        extractor = cv::ORB::create();
    }
    else if (descriptorType.compare("FREAK") == 0)
    {
        extractor = cv::xfeatures2d::FREAK::create();
    }
    else if (descriptorType.compare("AKAZE") == 0)
    {
        extractor = cv::AKAZE::create();
    }
    else if (descriptorType.compare("SIFT") == 0)
    {
        extractor = cv::SIFT::create();
    }
    else
    {
        std::cout << "no descriptorType is found" << "\n";
        exit(1);
    }

    // perform feature description
    double t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    double extractTime = 1000 * t / 1.0;

    return extractTime;
}
```

## MP.5 Descriptor Matching

Implement FLANN matching as well as k-nearest neighbor selection. Both methods must be selectable using the respective strings in the main function. 

Approach: To make FLANN matching as well as k-nearest neighbor selectable, in MidTermProject_Camera_Student.cpp, string `matcherType` is used to switch the matcher type between `MAT_BF` and `MAT_FLANN`, and `selectorType` is used to switch between `SEL_NN` and `SEL_KNN`. Then, `matcherType` and `selectorType` are passed to the function `matchDescriptors`, which is in matching2D_Student.cpp. 

```cs
/* MATCH KEYPOINT DESCRIPTORS */
// MidTermProject_Camera_Student.cpp

vector<cv::DMatch> matches;
string matcherType = "MAT_FLANN";        // MAT_BF, MAT_FLANN
string descriptorType = "DES_BINARY"; // DES_BINARY, DES_HOG
string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN

matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                    (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                    matches, descriptorType, matcherType, selectorType);
```

```cs
// matching2D_Student.cpp

void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
{
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (matcherType.compare("MAT_BF") == 0)
    {
        if (!(descriptorType.compare("DES_BINARY") == 0 || descriptorType.compare("DES_HOG") == 0))
        {
            std::cout << "no descriptorType is found." << "\n";
        }
        int normType = descriptorType.compare("DES_BINARY") == 0 ? cv::NORM_HAMMING : cv::NORM_L2;
        matcher = cv::BFMatcher::create(normType, crossCheck);
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
        if (descSource.type() != CV_32F || descRef.type() != CV_32F )
        { // OpenCV bug workaround : convert binary descriptors to floating point due to a bug in current OpenCV implementation
            descSource.convertTo(descSource, CV_32F);
            descRef.convertTo(descRef, CV_32F);
        }
        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    }
    else if
    {
        std::cout << "no matcher type is found." << "\n";
        exit(1);
    }

    ....
}
```

## MP.6 Descriptor Distance Ratio

Use the K-Nearest-Neighbor matching to implement the descriptor distance ratio test, which looks at the ratio of best vs. second-best match to decide whether to keep an associated pair of keypoints.

Approach: Apply KNN match in OpenCV and check the descriptor distance ratio between the best match and the second best match. If the ration is less than 0.8, the matching pair will be kept.

```cs
// matching2D_Student.cpp
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
{
    ....

    if (selectorType.compare("SEL_NN") == 0)
    { 
        // nearest neighbor (best match)
        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    { 
        // k nearest neighbors (k=2)
        vector<vector<cv::DMatch>> knn_matches;
        matcher->knnMatch(descSource, descRef, knn_matches, 2); // finds the 2 best matches
        // filter matches using descriptor distance ratio test
        double minDescDistRatio = 0.8;
        for (auto it = knn_matches.begin(); it != knn_matches.end(); ++it)
        {
            if ((*it)[0].distance < minDescDistRatio * (*it)[1].distance)
            {
                matches.push_back((*it)[0]);
            }
        }
    }
    else
    {
        std::cout << "no selectorType is found." << "\n";
        exit(1);
    }
}
```

## MP.7 Performance Evaluation 1

Your seventh task is to count the number of keypoints on the preceding vehicle for all 10 images and take note of the distribution of their neighborhood size. 

Do this for all the detectors you have implemented.

| Detectors    | Image_1 | Image_2 | Image_3 | Image_4 | Image_5 | Image_6 | Image_7 | Image_8 | Image_9 | Image_10 | 
|--------------|:-------:|:-------:|:-------:|:-------:|:-------:|:-------:|:-------:|:-------:|:-------:|:--------:|
| SHITOMASI    | 127     | 120     | 123     | 120     | 120     | 115     | 114     | 125     | 112     | 113      |
| HARRIS       | 17      | 14      | 18      | 21      | 26      | 43      | 18      | 31      | 26      | 34       |
| FAST         | 149     | 152     | 152     | 157     | 149     | 150     | 157     | 152     | 139     | 144      |
| BRISK        | 254     | 274     | 276     | 275     | 293     | 275     | 289     | 268     | 260     | 250      |
| ORB          | 91      | 102     | 106     | 113     | 109     | 124     | 129     | 127     | 124     | 125      |
| AKAZE        | 162     | 157     | 159     | 154     | 162     | 163     | 173     | 175     | 175     | 175      |
| SIFT         | 137     | 131     | 121     | 135     | 134     | 139     | 136     | 147     | 156     | 135      |

| Detectors    | Aveg. Neighborhood size | Note                                                    | 
|--------------|:-----------------------:|---------------------------------------------------------|
| SHITOMASI    | 4                       | evenly distributed in the center region                 |
| HARRIS       | 6                       | sparsely distributed. only a few keypoints are detected |
| FAST         | 7                       | evenly distributed                                      |
| BRISK        | 22.0445                 | larger size. overlapped with each other                 |
| ORB          | 56.0097                 | larger size. overlapped with each other                 |
| AKAZE        | 7.68091                 | evenly distributed on the edge of preceding vehicle     |
| SIFT         | 5.05154                 | most of keypoints's neighborhood size are small         |


## MP.8 Performance Evaluation 2

Your eighth task is to count the number of matched keypoints for all 10 images using all possible combinations of detectors and descriptors. 

In the matching step, use the `BF approach` with the descriptor distance ratio set to 0.8.

| Detectors/Descriptors | BRISK | BRIEF | ORB              | FREAK | AKAZE | SIFT |
|-----------------------|:-----:|:-----:|:----------------:|:-----:|:-----:|:----:|
| SHITOMASI             | 771   | 953   | 914              | 768   | N/A   | 936  |
| HARRIS                | 142   | 173   | 160              | 146   | N/A   | 163  |
| FAST                  | 904   | 1106  | 1090             | 887   | N/A   | 1054 |
| BRISK                 | 1545  | 1676  | 1480             | 1497  | N/A   | 1617 |
| ORB                   | 744   | 540   | 754              | 417   | N/A   | 756  |
| AKAZE                 | 1204  | 1257  | 1177             | 1181  | 1249  | 1263 |
| SIFT                  | 586   | 693   | OutOfMemoryError | 591   | N/A   | 790  |

[AKAZE descriptors](https://docs.opencv.org/master/d8/d30/classcv_1_1AKAZE.html) can only be used with KAZE or AKAZE keypoints.

## MP.9 Performance Evaluation 3

Your ninth task is to log the time it takes for keypoint detection and descriptor extraction. The results must be entered into a spreadsheet and based on this information you will then suggest the TOP3 detector / descriptor combinations as the best choice for our purpose of detecting keypoints on vehicles. 

Finally, in a short text, please justify your recommendation based on your observations and on the data you collected.

### Keypoint Detection Time (ms)

| Detectors/Descriptors | BRISK   | BRIEF   | ORB              | FREAK   | AKAZE   | SIFT    |
|-----------------------|:-------:|:-------:|:----------------:|:-------:|:-------:|:-------:|
| SHITOMASI             | 7.95837 | 13.1182 | 11.1887          | 6.88695 | N/A     | 8.81036 |
| HARRIS                | 8.35773 | 14.0026 | 12.6764          | 9.31824 | N/A     | 9.72301 |
| FAST                  | 0.75688 | 1.35862 | 1.47732          | 0.69830 | N/A     | 1.07794 |
| BRISK                 | 43.9083 | 43.9052 | 42.5106          | 43.1059 | N/A     | 43.9597 |
| ORB                   | 16.9654 | 19.1315 | 16.0469          | 16.0599 | N/A     | 14.1888 |
| AKAZE                 | 33.7575 | 37.4384 | 32.4252          | 31.1567 | 33.2999 | 32.6867 |
| SIFT                  | 55.4544 | 55.947  | OutOfMemoryError | 53.2511 | N/A     | 52.8841 |

### Descriptor Extraction Time (ms)

| Detectors/Descriptors | BRISK   | BRIEF    | ORB              | FREAK   | AKAZE   | SIFT    |
|-----------------------|:-------:|:--------:|:----------------:|:-------:|:-------:|:-------:|
| SHITOMASI             | 1.1703  | 1.19198  | 3.44498          | 25.5664 | N/A     | 11.9337 |
| HARRIS                | 0.36078 | 0.406347 | 5.28129          | 21.4451 | N/A     | 12.2801 |
| FAST                  | 1.34859 | 0.976105 | 4.8497           | 26.2078 | N/A     | 16.7838 |
| BRISK                 | 1.91132 | 1.25628  | 6.82906          | 18.8436 | N/A     | 14.1148 |
| ORB                   | 1.01503 | 0.912495 | 9.19884          | 26.1855 | N/A     | 23.3835 |
| AKAZE                 | 1.17559 | 0.968396 | 5.36003          | 19.7024 | 29.1429 | 12.7071 |
| SIFT                  | 1.04095 | 0.69018  | OutOfMemoryError | 18.0904 | N/A     | 43.7447 |

### TOP3 detector/descriptor

| Detectors/Descriptors | Matched Keypoints | Processing Time (ms) |
|-----------------------|:-----------------:|:--------------------:|
| FAST/BRIEF            | 1106              | 2.3347               |
| FAST/BRISK            | 904               | 2.105                |
| FAST/ORB              | 1090              | 6.327                | 

Top3 combinations of detector/descriptor not only have fair amount of matched keypoints but also faster processing speed comparing to other combinations.