# IQueryable

## `Aggregate`

시계열에 누산기 함수를 적용합니다.

```C#
TResult Aggregate<TSource>(IQueryable<TSource>, Expression<Func<TSource,TSource,TSource>>) 	
TResult Aggregate<TSource,TAccumulate>(IQueryable<TSource>, TAccumulate, Expression<Func<TAccumulate,TSource,TAccumulate>>)
TResult Aggregate<TSource,TAccumulate,TResult>(IQueryable<TSource>, TAccumulate, Expression<Func<TAccumulate,TSource,TAccumulate>>, Expression<Func<TAccumulate,TResult>>)
```

## `All`

시계열의 모든 요소가 조건을 충족하는지 여부를 결정합니다.

```C#
bool All<TSource>(IQueryable<TSource>, Expression<Func<TSource,Boolean>>) 	
```

## `Any`

시계열에 요소가 포함되어 있는지 여부를 확인합니다.

```C#
bool Any<TSource>(IQueryable<TSource>) 	
bool Any<TSource>(IQueryable<TSource>, Expression<Func<TSource,Boolean>>) 	
```

## `Append`

소스의 요소와 끝에 추가 된 지정된 요소를 포함하는 새 쿼리 가능한 시계열을 반환합니다.

```C#
IEnumerable<TSource> Append<TSource>(IQueryable<TSource>, TSource) 	
```

## `AsQueryable`

`IEnumerable`을 `IQueryable`로 변환합니다.

```C#
IQueryable           AsQueryable(IEnumerable) 	
IQueryable<TElement> AsQueryable<TElement>(IEnumerable<TElement>) 	
```

## `Average`

Computes the average of a sequence of numeric values.

```C#
decimal Average(IQueryable<Decimal>)
float   Average(IQueryable<Single>)
double  Average(IQueryable<Double>)
int     Average(IQueryable<Int32>) 	
long    Average(IQueryable<Int64>) 	
decimal Average(IQueryable<Nullable<Decimal>>) 	
float   Average(IQueryable<Nullable<Single>>)
double  Average(IQueryable<Nullable<Double>>) 	
int     Average(IQueryable<Nullable<Int32>>) 	
long    Average(IQueryable<Nullable<Int64>>) 	
decimal Average<TSource>(IQueryable<TSource>, Expression<Func<TSource,Decimal>>) 	
float   Average<TSource>(IQueryable<TSource>, Expression<Func<TSource,Single>>) 	
double  Average<TSource>(IQueryable<TSource>, Expression<Func<TSource,Double>>) 	
int     Average<TSource>(IQueryable<TSource>, Expression<Func<TSource,Int32>>) 	
long    Average<TSource>(IQueryable<TSource>, Expression<Func<TSource,Int64>>) 	
Nullable<Decimal> Average<TSource>(IQueryable<TSource>, Expression<Func<TSource,Nullable<Decimal>>>)
Nullable<Single>  Average<TSource>(IQueryable<TSource>, Expression<Func<TSource,Nullable<Single>>>)
Nullable<Double>  Average<TSource>(IQueryable<TSource>, Expression<Func<TSource,Nullable<Double>>>)
Nullable<Int32>   Average<TSource>(IQueryable<TSource>, Expression<Func<TSource,Nullable<Int32>>>)
Nullable<Int64>   Average<TSource>(IQueryable<TSource>, Expression<Func<TSource,Nullable<Int64>>>)
```

## `Cast`

Converts the elements of an `IQueryable` to the specified type.

```C#
IEnumerable<TResult> Cast<TResult>(IQueryable)
```

## `Concat`

Concatenates two sequences.

```C#
IEnumerable<TSource> Concat<TSource>(IQueryable<TSource>, IEnumerable<TSource>) 	
```

## `CopyToDataTable`

입력 `IEnumerable<T>` 개체가 있는 경우 `DataRow` 개체의 복사본이 포함 된 `DataTable`을 반환합니다.

```C#
DataTable CopyToDataTable<T>(IEnumerable<T>) 	
DataTable CopyToDataTable<T>(IEnumerable<T>, DataTable, LoadOption) 	
DataTable CopyToDataTable<T>(IEnumerable<T>, DataTable, LoadOption, FillErrorEventHandler) 
```

## `Contains`

시계열에 지정된 요소가 있는지 여부를 확인합니다.

```C#
bool Contains<TSource>(IQueryable<TSource>, TSource)
bool Contains<TSource>(IQueryable<TSource>, TSource, IEqualityComparer<TSource>)
```

## `Count`

시계열 안의 요소수를 반환합니다.

```C#
int Count<TSource>(IQueryable<TSource>)
int Count<TSource>(IQueryable<TSource>, Expression<Func<TSource,Boolean>>)
```

## `DefaultIfEmpty`

`IEnumerable<T>`의 요소를 반환하거나 시계열이 비어있는 경우 기본값 싱글톤 컬렉션을 반환합니다.

```C#
IEnumerable<TSource> DefaultIfEmpty<TSource>(IQueryable<TSource>) 	
IEnumerable<TSource> DefaultIfEmpty<TSource>(IQueryable<TSource>, TSource) 	
```

## `Distinct`

고유한 요소를 반환합니다.

```C#
IEnumerable<TSource> Distinct<TSource>(IQueryable<TSource>)
IEnumerable<TSource> Distinct<TSource>(IQueryable<TSource>, IEqualityComparer<TSource>)
```

## `ElementAt`, `ElementAtOrDefault`

시계열에서 고유한 요소를 반환합니다.

```C#
TSource ElementAt<TSource>(IQueryable<TSource>, Int32)
TSource ElementAtOrDefault<TSource>(IQueryable<TSource>, Int32)
```

## `Except`

기본 비교자를 사용하여 값을 비교하여 두 시계열의 집합 차이를 생성합니다.

```C#
IEnumerable<TSource> Except<TSource>(IQueryable<TSource>, IEnumerable<TSource>)
IEnumerable<TSource> Except<TSource>(IQueryable<TSource>, IEnumerable<TSource>, IEqualityComparer<TSource>)
```

## `First`, `FirstOrDefault`

시계열의 첫번째 요소를 반환합니다.

```C#
TSource First<TSource>(IQueryable<TSource>)
TSource First<TSource>(IQueryable<TSource>, Expression<Func<TSource,Boolean>>)
TSource FirstOrDefault<TSource>(IQueryable<TSource>)
TSource FirstOrDefault<TSource>(IQueryable<TSource>, Expression<Func<TSource,Boolean>>)
```

## `GroupBy`


시계열의 요소를 그룹화합니다.


```C#
IEnumerable<TResult> GroupBy<TSource,TKey>(IQueryable<TSource>, Expression<Func<TSource,TKey>>) 	
IEnumerable<TResult> GroupBy<TSource,TKey>(IQueryable<TSource>, Expression<Func<TSource,TKey>>, IEqualityComparer<TKey>) 	
IEnumerable<TResult> GroupBy<TSource,TKey,TElement>(IQueryable<TSource>, Expression<Func<TSource,TKey>>, Expression<Func<TSource,TElement>>) 	
IEnumerable<TResult> GroupBy<TSource,TKey,TElement>(IQueryable<TSource>, Expression<Func<TSource,TKey>>, Expression<Func<TSource,TElement>>, IEqualityComparer<TKey>) 	
IEnumerable<TResult> GroupBy<TSource,TKey,TResult>(IQueryable<TSource>, Expression<Func<TSource,TKey>>, Expression<Func<TKey,IEnumerable<TSource>,TResult>>) 	
IEnumerable<TResult> GroupBy<TSource,TKey,TResult>(IQueryable<TSource>, Expression<Func<TSource,TKey>>, Expression<Func<TKey,IEnumerable<TSource>,TResult>>, IEqualityComparer<TKey>) 	
IEnumerable<TResult> GroupBy<TSource,TKey,TElement,TResult>(IQueryable<TSource>, Expression<Func<TSource,TKey>>, Expression<Func<TSource,TElement>>, Expression<Func<TKey,IEnumerable<TElement>,TResult>>) 	
IEnumerable<TResult> GroupBy<TSource,TKey,TElement,TResult>(IQueryable<TSource>, Expression<Func<TSource,TKey>>, Expression<Func<TSource,TElement>>, Expression<Func<TKey,IEnumerable<TElement>,TResult>>, IEqualityComparer<TKey>)
```

## `GroupJoin`

키 동등성을 기준으로 두 시계열의 요소를 상관시키고 결과를 그룹화합니다.

```C#
IEnumerable<TResult> GroupJoin<TOuter,TInner,TKey,TResult>(IQueryable<TOuter>, IEnumerable<TInner>, Expression<Func<TOuter,TKey>>, Expression<Func<TInner,TKey>>, Expression<Func<TOuter,IEnumerable<TInner>,TResult>>) 	
IEnumerable<TResult> GroupJoin<TOuter,TInner,TKey,TResult>(IQueryable<TOuter>, IEnumerable<TInner>, Expression<Func<TOuter,TKey>>, Expression<Func<TInner,TKey>>, Expression<Func<TOuter,IEnumerable<TInner>,TResult>>, IEqualityComparer<TKey>) 	
```

## `Intersect`

두 시계열의 교차 집합을 생성합니다.

```C#
IEnumerable<TResult> Intersect<TSource>(IQueryable<TSource>, IEnumerable<TSource>) 	
IEnumerable<TResult> Intersect<TSource>(IQueryable<TSource>, IEnumerable<TSource>, IEqualityComparer<TSource>) 	
```

## `Join`

일치하는 키를 기반으로 두 열시계의 요소를 상호 연관시킵니다.

```C#
TResult Join<TOuter,TInner,TKey,TResult>(IQueryable<TOuter>, IEnumerable<TInner>, Expression<Func<TOuter,TKey>>, Expression<Func<TInner,TKey>>, Expression<Func<TOuter,TInner,TResult>>) 	
TResult Join<TOuter,TInner,TKey,TResult>(IQueryable<TOuter>, IEnumerable<TInner>, Expression<Func<TOuter,TKey>>, Expression<Func<TInner,TKey>>, Expression<Func<TOuter,TInner,TResult>>, IEqualityComparer<TKey>) 	
```

## `Last`, `LastOrDefault`

열시계의 마지막 요소를 반환합니다.

```C#
TSource Last<TSource>(IQueryable<TSource>)
TSource Last<TSource>(IQueryable<TSource>, Expression<Func<TSource,Boolean>>)
TSource LastOrDefault<TSource>(IQueryable<TSource>)
TSource LastOrDefault<TSource>(IQueryable<TSource>, Expression<Func<TSource,Boolean>>)
```

## `LongCount`

열시계의 총 요소 수를 나타내는 Int64를 반환합니다.

```C#
long LongCount<TSource>(IQueryable<TSource>)
long LongCount<TSource>(IQueryable<TSource>, Expression<Func<TSource,Boolean>>)
```

## `Max`

제네릭 IQueryable <T>의 각 요소에 대해 프로젝션 함수를 호출하고 최대 결과 값을 반환합니다.

```C#
TSource Max<TSource>(IQueryable<TSource>) 	
TSource Max<TSource,TResult>(IQueryable<TSource>, Expression<Func<TSource,TResult>>) 	
```

## `Min`

제네릭 `IQueryable<T>`의 각 요소에 대해 프로젝션 함수를 호출하고 최소 결과 값을 반환합니다.

```C#
TSource Min<TSource>(IQueryable<TSource>) 	
TSource Min<TSource,TResult>(IQueryable<TSource>, Expression<Func<TSource,TResult>>) 	
```

## `OfType`

지정된 형식에 따라 `IQueryable`의 요소를 필터링합니다.

```C#
IEnumerable<TResult> OfType<TResult>(IQueryable) 	
```

## `OrderBy`, `OrderByDescending`

열시계의 요소를 키에 따라 정렬합니다.

```C#
IOrderedEnumerable<TSource> OrderBy<TSource,TKey>(IQueryable<TSource>, Expression<Func<TSource,TKey>>) 	
IOrderedEnumerable<TSource> OrderBy<TSource,TKey>(IQueryable<TSource>, Expression<Func<TSource,TKey>>, IComparer<TKey>) 	
IOrderedEnumerable<TSource> OrderByDescending<TSource,TKey>(IQueryable<TSource>, Expression<Func<TSource,TKey>>) 	
IOrderedEnumerable<TSource> OrderByDescending<TSource,TKey>(IQueryable<TSource>, Expression<Func<TSource,TKey>>, IComparer<TKey>) 	
```

## `Prepend`

소스의 요소와 시작 부분에 추가 된 지정된 요소를 포함하는 새 쿼리 가능한 열시계를 반환합니다.

```C#
IEnumerable<TSource> Prepend<TSource>(IQueryable<TSource>, TSource) 	
```

## `Reverse`

열시계에서 요소의 순서를 반전합니다.

```C#
IEnumerable<TSource> Reverse<TSource>(IQueryable<TSource>)
```

## `Select`

열시계의 각 요소를 새 형식으로 투영합니다.

```C#
IEnumerable<TResult> Select<TSource,TResult>(IQueryable<TSource>, Expression<Func<TSource,TResult>>)
IEnumerable<TResult> Select<TSource,TResult>(IQueryable<TSource>, Expression<Func<TSource,Int32,TResult>>)
```

## `SelectMany`

열시계의 각 요소를 IEnumerable <T>에 투영하고 결과 열시계를 하나의 열시계로 평면화합니다.

```
IEnumerable<TResult> SelectMany<TSource,TResult>(IQueryable<TSource>, Expression<Func<TSource,IEnumerable<TResult>>>) 	
IEnumerable<TResult> SelectMany<TSource,TResult>(IQueryable<TSource>, Expression<Func<TSource,Int32,IEnumerable<TResult>>>) 	
IEnumerable<TResult> SelectMany<TSource,TCollection,TResult>(IQueryable<TSource>, Expression<Func<TSource,IEnumerable<TCollection>>>, Expression<Func<TSource,TCollection,TResult>>) 	
IEnumerable<TResult> SelectMany<TSource,TCollection,TResult>(IQueryable<TSource>, Expression<Func<TSource,Int32,IEnumerable<TCollection>>>, Expression<Func<TSource,TCollection,TResult>>) 	
```

## `SequenceEqual`

두 열시계가 같은지 여부를 확인합니다.

```C#
bool SequenceEqual<TSource>(IQueryable<TSource>, IEnumerable<TSource>)
bool SequenceEqual<TSource>(IQueryable<TSource>, IEnumerable<TSource>, IEqualityComparer<TSource>)
```

## `Single`, `SingleOrDefault`

열시계의 유일한 요소를 반환하고 열시계에 정확히 하나의 요소가 없으면 예외를 throw합니다.

```C#
TSource<TSource> Single<TSource>(IQueryable<TSource>) 	
TSource<TSource> Single<TSource>(IQueryable<TSource>, Expression<Func<TSource,Boolean>>) 	
TSource<TSource> SingleOrDefault<TSource>(IQueryable<TSource>) 	
TSource<TSource> SingleOrDefault<TSource>(IQueryable<TSource>, Expression<Func<TSource,Boolean>>) 	
```

## `Skip`, `SkipLast`, `SkipWhile`

열시계에서 지정된 수의 요소를 우회 한 다음 나머지 요소를 반환합니다.

```C#
IQueryable<TSource> Skip<TSource>(IQueryable<TSource>, Int32)
IQueryable<TSource> SkipLast<TSource>(IQueryable<TSource>, Int32)
IQueryable<TSource> SkipWhile<TSource>(IQueryable<TSource>, Expression<Func<TSource,Boolean>>)
IQueryable<TSource> SkipWhile<TSource>(IQueryable<TSource>, Expression<Func<TSource,Int32,Boolean>>)
```

## `Sum`

열시계의 합을 계산합니다.

```C#
decimal Sum(IQueryable<Decimal>)
double  Sum(IQueryable<Double>)
int     Sum(IQueryable<Int32>)
long    Sum(IQueryable<Int64>)
float   Sum(IQueryable<Single>)
decimal Sum(IQueryable<Nullable<Decimal>>)
double  Sum(IQueryable<Nullable<Double>>)
int     Sum(IQueryable<Nullable<Int32>>)
long    Sum(IQueryable<Nullable<Int64>>)
float   Sum(IQueryable<Nullable<Single>>)
decimal Sum<TSource>(IQueryable<TSource>, Expression<Func<TSource,Decimal>>) 	
double  Sum<TSource>(IQueryable<TSource>, Expression<Func<TSource,Double>>) 	
int     Sum<TSource>(IQueryable<TSource>, Expression<Func<TSource,Int32>>) 	
long    Sum<TSource>(IQueryable<TSource>, Expression<Func<TSource,Int64>>) 	
float   Sum<TSource>(IQueryable<TSource>, Expression<Func<TSource,Single>>) 	
decimal Sum<TSource>(IQueryable<TSource>, Expression<Func<TSource,Nullable<Decimal>>>) 	
double  Sum<TSource>(IQueryable<TSource>, Expression<Func<TSource,Nullable<Double>>>) 	
int     Sum<TSource>(IQueryable<TSource>, Expression<Func<TSource,Nullable<Int32>>>) 	
long    Sum<TSource>(IQueryable<TSource>, Expression<Func<TSource,Nullable<Int64>>>) 	
float   Sum<TSource>(IQueryable<TSource>, Expression<Func<TSource,Nullable<Single>>>) 	
```

## `Take`, `TakeWhile`

지정된 수의 연속 요소를 반환합니다.

```C#
IQueryable<TSource> Take<TSource>(IQueryable<TSource>, Int32)
IQueryable<TSource> TakeLast<TSource>(IQueryable<TSource>, Int32)
IQueryable<TSource> TakeWhile<TSource>(IQueryable<TSource>, Expression<Func<TSource,Boolean>>)
IQueryable<TSource> TakeWhile<TSource>(IQueryable<TSource>, Expression<Func<TSource,Int32,Boolean>>)
```

## `ThenBy`, `ThenByDescending`

키에 따라 오름차순으로 순서대로 요소의 후속 순서를 수행합니다.

```C#
IOrderedQueryable<TSource> ThenBy<TSource,TKey>(IOrderedQueryable<TSource>, Expression<Func<TSource,TKey>>) 	
IOrderedQueryable<TSource> ThenBy<TSource,TKey>(IOrderedQueryable<TSource>, Expression<Func<TSource,TKey>>, IComparer<TKey>) 	
IOrderedQueryable<TSource> ThenByDescending<TSource,TKey>(IOrderedQueryable<TSource>, Expression<Func<TSource,TKey>>) 	
IOrderedQueryable<TSource> ThenByDescending<TSource,TKey>(IOrderedQueryable<TSource>, Expression<Func<TSource,TKey>>, IComparer<TKey>) 	
```

## `ToImmutable`

지정된 컬렉션에서 변경 불가능한 항목을 만듭니다.

```C#
ImmutableArray<TSource>      ToImmutableArray<TSource>(IEnumerable<TSource>)
ImmutableDictionary<TSource> ToImmutableDictionary<TSource,TKey>(IEnumerable<TSource>, Func<TSource,TKey>) 	
ImmutableDictionary<TSource> ToImmutableDictionary<TSource,TKey>(IEnumerable<TSource>, Func<TSource,TKey>, IEqualityComparer<TKey>) 	
ImmutableDictionary<TSource> ToImmutableDictionary<TSource,TKey,TValue>(IEnumerable<TSource>, Func<TSource,TKey>, Func<TSource,TValue>) 	
ImmutableDictionary<TSource> ToImmutableDictionary<TSource,TKey,TValue>(IEnumerable<TSource>, Func<TSource,TKey>, Func<TSource,TValue>, IEqualityComparer<TKey>) 	
ImmutableDictionary<TSource> ToImmutableDictionary<TSource,TKey,TValue>(IEnumerable<TSource>, Func<TSource,TKey>, Func<TSource,TValue>, IEqualityComparer<TKey>, IEqualityComparer<TValue>) 	
ImmutableHashSet<TSource>    ToImmutableHashSet<TSource>(IEnumerable<TSource>) 	
ImmutableHashSet<TSource>    ToImmutableHashSet<TSource>(IEnumerable<TSource>, IEqualityComparer<TSource>) 	
ImmutableList<TSource>       ToImmutableList<TSource>(IEnumerable<TSource>)
ImmutableSortedDictionary<TSource> ToImmutableSortedDictionary<TSource,TKey,TValue>(IEnumerable<TSource>, Func<TSource,TKey>, Func<TSource,TValue>) 	
ImmutableSortedDictionary<TSource> ToImmutableSortedDictionary<TSource,TKey,TValue>(IEnumerable<TSource>, Func<TSource,TKey>, Func<TSource,TValue>, IComparer<TKey>) 	
ImmutableSortedDictionary<TSource> ToImmutableSortedDictionary<TSource,TKey,TValue>(IEnumerable<TSource>, Func<TSource,TKey>, Func<TSource,TValue>, IComparer<TKey>, IEqualityComparer<TValue>) 	
ImmutableSortedSet<TSource>        ToImmutableSortedSet<TSource>(IEnumerable<TSource>) 	
ImmutableSortedSet<TSource>        ToImmutableSortedSet<TSource>(IEnumerable<TSource>, IComparer<TSource>) 	
```

## `Union`

기본 비교자를 사용하여 두 열시계의 합집합을 생성합니다.

```C#
IQueryable<TResult> Union<TSource>(IQueryable<TSource>, IEnumerable<TSource>)
IQueryable<TResult> Union<TSource>(IQueryable<TSource>, IEnumerable<TSource>, IEqualityComparer<TSource>) 	
```

## `Where`

술어를 기반으로 일련의 값을 필터링합니다.

```C#
IQueryable<TSource> Where<TSource>(IQueryable<TSource>, Expression<Func<TSource,Boolean>>) 	
IQueryable<TSource> Where<TSource>(IQueryable<TSource>, Expression<Func<TSource,Int32,Boolean>>) 	
```

## `Zip`

지정된 두 열시계의 요소로 튜플 열시계를 생성합니다.

```C#
IQueryable<TSource> Zip<TFirst,TSecond>(IQueryable<TFirst>, IEnumerable<TSecond>)
IQueryable<TSource> Zip<TFirst,TSecond,TResult>(IQueryable<TFirst>, IEnumerable<TSecond>, Expression<Func<TFirst,TSecond,TResult>>)
```


소스 컬렉션에있는 모든 노드의 조상을 포함하는 요소 컬렉션을 반환합니다.


```C#
IEnumerable<XElement> Ancestors<T>(IEnumerable<T>)
IEnumerable<XElement> Ancestors<T>(IEnumerable<T>, XName)
```

소스 컬렉션에있는 모든 요소 및 문서의 하위 요소를 포함하는 요소 컬렉션을 반환합니다.

```C#
IEnumerable<XElement> Descendants<T>(IEnumerable<T>)
IEnumerable<XElement> Descendants<T>(IEnumerable<T>, XName)
```

소스 컬렉션에있는 모든 문서 및 요소의 하위 노드 컬렉션을 반환합니다.

```C#
IEnumerable<XElement> DescendantNodes<T>(IEnumerable<T>)
```

소스 컬렉션에있는 모든 요소 및 문서의 자식 요소 컬렉션을 반환합니다.

```C#
IEnumerable<XElement> Elements<T>(IEnumerable<T>)
IEnumerable<XElement> Elements<T>(IEnumerable<T>, XName)
```

문서 순서로 정렬 된 소스 컬렉션의 모든 노드를 포함하는 노드 컬렉션을 반환합니다.

```C#
IEnumerable<T> InDocumentOrder<T>(IEnumerable<T>)
```

소스 컬렉션에있는 모든 문서 및 요소의 자식 노드 컬렉션을 반환합니다.

```C#
IEnumerable<XNode> Nodes<T>(IEnumerable<T>)
```

부모 노드에서 원본 컬렉션의 모든 노드를 제거합니다.

```C#
IEnumerable<XNode> Remove<T>(IEnumerable<T>)
```

