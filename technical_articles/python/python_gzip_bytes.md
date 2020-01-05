자주 사용되는 보일러플레이트

# Python에서 numpy array를 gzip에 저장

`numpy.array`가 있습니다.

```python
nparray1 = np.array([...])
```

`numpy.array`가 여러개 있을 수도 있죠.

```python
nparray2 = np.array([...])
```

파이썬 `bytearray` 객체를 생성합니다. 이 객체에 바이너리로 임시 보관 될 것입니다.

```python
bytes = bytearray()
```

`bytes`에 `nparray1`를 바이트로 변환합니다.

```python
bytes += nparray1.tobytes()
bytes += nparray2.tobytes()
```

이제 `bytes`를 저장 할 수 있습니다. 먼저 gzip 파일을 생성하고 저장합니다.

```python
with gzip.open(파일명 + "gz", "wb") as gzf:
    gzf.write(bytes)
```

쉽죠?

