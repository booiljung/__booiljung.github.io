자주 사용되는 보일러플레이트

# Python: MNIST 처럼 IDX1이나 IDX3파일 만들기

Yann LeCun 교수가 만든 필기체 이미지 데이터 MNIST는 딥러닝 뉴비들을 위한 좋은 데이터셋입니다. MNIST 데이터셋은 IDX라는 파일 포맷을 되어 있고, 데이터 구조는 [MNIST 페이지](http://yann.lecun.com/exdb/mnist/)에 소개 되어 있습니다.

##### TRAINING SET LABEL FILE (train-labels-idx1-ubyte):

```
[offset] [type]          [value]          [description]
0000     32 bit integer  0x00000801(2049) magic number (MSB first)
0004     32 bit integer  60000            number of items
0008     unsigned byte   ??               label
0009     unsigned byte   ??               label
........
xxxx     unsigned byte   ??               label

The labels values are 0 to 9. 
```

##### TRAINING SET IMAGE FILE (train-images-idx3-ubyte):

```
[offset] [type]     [value]     [description]` 
`0000   32 bit integer 0x00000803(2051) magic number` 
`0004   32 bit integer 60000      number of images` 
`0008   32 bit integer 28        number of rows` 
`0012   32 bit integer 28        number of columns` 
`0016   unsigned byte  ??        pixel` 
`0017   unsigned byte  ??        pixel` 
`........` 
`xxxx   unsigned byte  ??        pixel
```

##### TEST SET LABEL FILE (t10k-labels-idx1-ubyte):

```
[offset] [type]          [value]          [description]
0000     32 bit integer  0x00000801(2049) magic number (MSB first)
0004     32 bit integer  10000            number of items
0008     unsigned byte   ??               label
0009     unsigned byte   ??               label
........
xxxx     unsigned byte   ??               label
```

##### TEST SET IMAGE FILE (t10k-images-idx3-ubyte):

```
[offset] [type]          [value]          [description]
0000     32 bit integer  0x00000803(2051) magic number
0004     32 bit integer  10000            number of images
0008     32 bit integer  28               number of rows
0012     32 bit integer  28               number of columns
0016     unsigned byte   ??               pixel
0017     unsigned byte   ??               pixel
........
xxxx     unsigned byte   ??               pixel
```

MNIST 데이터셋은 많은 딥러닝 라이브러리들이 지원하여, 이 포맷을 사용하면, 적은 수고로도 새 데이터셋을 활용 할 수 있게 되는데, 자신이 만든 데이터셋을 적은 노력으로도 생성 할 수 있어야 합니다.

IDX 파일 포맷에 대한 자료를 찾아보면 R이 지원하는 포맷이라고 나옵니다. R를 사용하여 이미지를 저장하면 해결 되겠지만,  Python을 사용하여 저장할 수 있을까 자료를 찾아 봤습니다. 그리고 [이 예제](https://github.com/davidflanagan/notMNIST-to-MNIST/blob/master/convert_to_mnist_format.py)를 찾을 수 있었습니다.

아래 코드는 매직 넘버가 `0x0801`으로 라벨 데이터를 저장합니다.

```python
def write_labeldata(labeldata, outputfile):
  header = numpy.array([0x0801, len(labeldata)], dtype='>i4')
  with open(outputfile, "wb") as f:
    f.write(header.tobytes())
    f.write(labeldata.tobytes())
```

다음 코드는 매직 넘버가 `0x0803`으로 이미지 데이터를 저장합니다.

```python
def write_imagedata(imagedata, outputfile):
  header = numpy.array([0x0803, len(imagedata), 28, 28], dtype='>i4')
  with open(outputfile, "wb") as f:
    f.write(header.tobytes())
    f.write(imagedata.tobytes())
```

이 코드들에서 `labeldata`와 `imagedata`는 `numpy.array` 입니다.

라벨 데이터는

```
data = [
	1, 2, 3, ... 1, 2, 3, 5, 
]
```

RGB 이미지 데이터는

```
data = [
	[
        [pixel, pixel, ... pixel, pixel], // 이미지1 R 채널
        [pixel, pixel, ... pixel, pixel],
        ...
        [pixel, pixel, ... pixel, pixel],
        [pixel, pixel, ... pixel, pixel],
    ],
    [
        [pixel, pixel, ... pixel, pixel], // 이미지1 G 채널
        [pixel, pixel, ... pixel, pixel],
        ...
        [pixel, pixel, ... pixel, pixel],
        [pixel, pixel, ... pixel, pixel],
    ],
    [
        [pixel, pixel, ... pixel, pixel], // 이미지1 B 채널
        [pixel, pixel, ... pixel, pixel],
        ...
        [pixel, pixel, ... pixel, pixel],
        [pixel, pixel, ... pixel, pixel],
    ],
    ...
	[
        [pixel, pixel, ... pixel, pixel], // 이미지N R 채널
        [pixel, pixel, ... pixel, pixel],
        ...
        [pixel, pixel, ... pixel, pixel],
        [pixel, pixel, ... pixel, pixel],
    ],
    [
        [pixel, pixel, ... pixel, pixel], // 이미지N G 채널
        [pixel, pixel, ... pixel, pixel],
        ...
        [pixel, pixel, ... pixel, pixel],
        [pixel, pixel, ... pixel, pixel],
    ],
    [
        [pixel, pixel, ... pixel, pixel], // 이미지N B 채널
        [pixel, pixel, ... pixel, pixel],
        ...
        [pixel, pixel, ... pixel, pixel],
        [pixel, pixel, ... pixel, pixel],
    ],
]
```

그레이 스케일 이미지 데이터는

```
data = [
    [
    	[pixel, pixel, ... pixel, pixel], // 이미지1
        [pixel, pixel, ... pixel, pixel],
        ...
        [pixel, pixel, ... pixel, pixel],
        [pixel, pixel, ... pixel, pixel],
    ],
    [
        [pixel, pixel, ... pixel, pixel], // 이미지2
        [pixel, pixel, ... pixel, pixel],
        ...
        [pixel, pixel, ... pixel, pixel],
        [pixel, pixel, ... pixel, pixel],
    ],
	...
    [pixel, pixel, ... pixel, pixel], // 이미지 n
    [pixel, pixel, ... pixel, pixel],
    ...
    [pixel, pixel, ... pixel, pixel],
    [pixel, pixel, ... pixel, pixel],
]
```



으로 수집하여,

```
np.asarray(data)
```

를 통해 `np.array`로 변환 한다.

## 참조

- [MNIST, Yann LeCun](http://yann.lecun.com/exdb/mnist/)
- [davidflanagan/notMNIST-to-MNIST/convert_to_mnist_format.py](https://github.com/davidflanagan/notMNIST-to-MNIST/blob/master/convert_to_mnist_format.py)