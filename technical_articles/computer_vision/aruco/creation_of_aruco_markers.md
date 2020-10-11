# Creation of ArUco markers

ArUco 마커를 생성하려면 opencv_contib에서 지원하는 aruco 모듈을 사용할 수 있습니다.

opencv 빌드시 다음 definition을 포함하도록 합니다.

```cmake
-D BUILD_EXAMPLES=ON
-D OPENCV_EXTRA_MODULES_PATH=<opencv_contrib_directory>
```

빌드하면

<opencv_build_directory>/bin에 예제 바이너리 파일들이 `example_<folder_name>_<file_name>`이름으로 생성됩니다.

저의 경우 

```sh
sudo cp <opencv_build_directory>/bin/* /usr/local/bin
```

하여 사용합니다.

create_marker 사용방법은 아래와 같습니다.

`-d`: 미리 정의된 dictionary의 인덱스를 지정합니다.

[opencv_contrib/modules/aruco/inlcude/opencv2/aruco/dictionary.hpp](https://github.com/opencv/opencv_contrib/blob/master/modules/aruco/include/opencv2/aruco/dictionary.hpp) 파일에 다음과 같이 정의되어 있습니다.

```
DICT_4X4_50,
DICT_4X4_100,
DICT_4X4_250,
DICT_4X4_1000,
DICT_5X5_50,
DICT_5X5_100,
DICT_5X5_250,
DICT_5X5_1000,
DICT_6X6_50,
DICT_6X6_100,
DICT_6X6_250,
DICT_6X6_1000,
DICT_7X7_50,
DICT_7X7_100,
DICT_7X7_250,
DICT_7X7_1000,
DICT_ARUCO_ORIGINAL,
DICT_APRILTAG_16h5,
DICT_APRILTAG_25h9,
DICT_APRILTAG_36h10,
DICT_APRILTAG_36h11
```

`DICT_4X4_50` 여기서 `4X4`는 마커의 칸 캣수, `50`은 마커의 크기를 나타냅니다.

실제 마커의 ID는 [opencv_contrib/modules/aruco/src/predefined_dictionaries.hpp](https://github.com/opencv/opencv_contrib/blob/master/modules/aruco/src/predefined_dictionaries.hpp)에 정의되어 있습니다.

`-id`: dictionary에 속한 마커들 중 사용할 마커의 id 이며, 인덱스로 지정 합니다.

`-ms`: 마커 이미지의 크기를 픽셀 단위로 지정합니다. 200x200은 `200`으로 지정합니다.

`-bb`: 테두리 영역의 크기를 지정합니다.

예를 들면

```
example_aruco_create_marker -d=0 --id=0 --ms=200 --bb=1 marker_d0_id0_ms200_bb1.png
example_aruco_create_marker -d=0 --id=1 --ms=200 --bb=1 marker_d0_id1_ms200_bb1.png
example_aruco_create_marker -d=0 --id=2 --ms=200 --bb=1 marker_d0_id2_ms200_bb1.png
example_aruco_create_marker -d=4 --id=0 --ms=200 --bb=1 marker_d1_id0_ms200_bb1.png
example_aruco_create_marker -d=4 --id=1 --ms=200 --bb=1 marker_d1_id1_ms200_bb1.png
example_aruco_create_marker -d=4 --id=2 --ms=200 --bb=1 marker_d1_id2_ms200_bb1.png
example_aruco_create_marker -d=8 --id=0 --ms=200 --bb=1 marker_d2_id0_ms200_bb1.png
example_aruco_create_marker -d=8 --id=1 --ms=200 --bb=1 marker_d2_id1_ms200_bb1.png
example_aruco_create_marker -d=8 --id=2 --ms=200 --bb=1 marker_d2_id2_ms200_bb1.png
```

