# Provider

A mixture between dependency injection (DI) and state management, built with widgets for widgets.

It purposefully uses widgets for DI/state management instead of dart-only classes like `Stream`. The reason is, widgets are very simple yet robust and scalable.

By using widgets for state management, `provider` can guarantee:

- maintainability, through a forced uni-directional data-flow
- testability/composability, since it is always possible to mock/override a value
- robustness, as it is harder to forget to handle the update scenario of a model/widget

To read more about `provider`, see the [documentation](https://pub.dev/documentation/provider/latest/).

## Migration from v2.0.0 to v3.0.0 [#](https://pub.dev/packages/provider#migration-from-v200-to-v300)

- Providers can no longer be instantiated with `const`.
- `Provider` now throws if used with a `Listenable`/`Stream`. Consider using `ListenableProvider`/`StreamProvider` instead. Alternatively, this exception can be disabled by setting `Provider.debugCheckInvalidValueType` to `null` like so:

```dart
void main() {
  Provider.debugCheckInvalidValueType = null;

  runApp(MyApp());
}
```

- All `XXProvider.value` constructors now use `value` as parameter name.

Before:

```dart
ChangeNotifierProvider.value(notifier: myNotifier),
```

After:

```dart
ChangeNotifierProvider.value(value: myNotifier),
```

- `StreamProvider`'s default constructor now builds a `Stream` instead of a `StreamController`. The previous behavior has been moved to the named constructor `StreamProvider.controller`.

Before:

```dart
StreamProvider(builder: (_) => StreamController<int>()),
```

After:

```dart
StreamProvider.controller(builder: (_) => StreamController<int>()),
```

## Usage [#](https://pub.dev/packages/provider#usage)

### Exposing a value [#](https://pub.dev/packages/provider#exposing-a-value)

To expose a variable using `provider`, wrap any widget into one of the provider widgets from this package and pass it your variable. Then, all descendants of the newly added provider widget can access this variable.

A simple example would be to wrap the entire application into a `Provider` widget and pass it our variable:

```dart
Provider<String>.value(
  value: 'Hello World',
  child: MaterialApp(
    home: Home(),
  )
)
```

Alternatively, for complex objects, most providers expose a constructor that takes a function to create the value. The provider will call that function only once, when inserting the widget in the tree, and expose the result. This is perfect for exposing a complex object that never changes over time without writing a `StatefulWidget`.

The following creates and exposes a `MyComplexClass`. And in the event where `Provider` is removed from the widget tree, the instantiated `MyComplexClass` will be disposed.

```dart
Provider<MyComplexClass>(
  builder: (context) => MyComplexClass(),
  dispose: (context, value) => value.dispose()
  child: SomeWidget(),
)
```

### Reading a value [#](https://pub.dev/packages/provider#reading-a-value)

The easiest way to read a value is by using the static method `Provider.of<T>(BuildContext context)`.

This method will look up in the widget tree starting from the widget associated with the `BuildContext` passed and it will return the nearest variable of type `T` found (or throw if nothing is found).

Combined with the first example of [exposing a value](https://pub.dev/packages/provider#exposing-a-value), this widget will read the exposed `String` and render "Hello World."

```dart
class Home extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return Text(
      /// Don't forget to pass the type of the object you want to obtain to `Provider.of`!
      Provider.of<String>(context)
    );
  }
}
```

Alternatively instead of using `Provider.of`, we can use `Consumer` and `Selector`.

These can be useful for performance optimizations or when it is difficult to obtain a `BuildContext` descendant of the provider.

See the [FAQ](https://github.com/rrousselGit/provider#my-widget-rebuilds-too-often-what-can-i-do) or the documentation of [Consumer](https://pub.dev/documentation/provider/latest/provider/Consumer-class.html) and [Selector](https://pub.dev/documentation/provider/latest/provider/Selector-class.html) for more information.

### `MultiProvider` [#](https://pub.dev/packages/provider#multiprovider)

When injecting many values in big applications, `Provider` can rapidly become pretty nested:

```dart
Provider<Foo>.value(
  value: foo,
  child: Provider<Bar>.value(
    value: bar,
    child: Provider<Baz>.value(
      value: baz,
      child: someWidget,
    )
  )
)
```

In that situation, we can use `MultiProvider` to improve the readability:

```dart
MultiProvider(
  providers: [
    Provider<Foo>.value(value: foo),
    Provider<Bar>.value(value: bar),
    Provider<Baz>.value(value: baz),
  ],
  child: someWidget,
)
```

The behavior of both examples is strictly the same. `MultiProvider` only changes the appearance of the code.

### `ProxyProvider`[#](https://pub.dev/packages/provider#proxyprovider)

Since the 3.0.0, there is a new kind of provider: `ProxyProvider`.

`ProxyProvider` is a provider that combines multiple values from other providers into a new object, and sends the result to `Provider`.

That new object will then be updated whenever one of the providers it depends on updates.

The following example uses `ProxyProvider` to build translations based on a counter coming from another provider.

```dart
Widget build(BuildContext context) {
  return MultiProvider(
    providers: [
      ChangeNotifierProvider(builder: (_) => Counter()),
      ProxyProvider<Counter, Translations>(
        builder: (_, counter, __) => Translations(counter.value),
      ),
    ],
    child: Foo(),
  );
}

class Translations {
  const Translations(this._value);

  final int _value;

  String get title => 'You clicked $_value times';
}
```

It comes under multiple variations, such as:

- `ProxyProvider` vs `ProxyProvider2` vs `ProxyProvider3`, ...

  That digit after the class name is the number of other providers that `ProxyProvider` depends on.

- `ProxyProvider` vs `ChangeNotifierProxyProvider` vs `ListenableProxyProvider`, ...

  They all work similarly, but instead of sending the result into a `Provider`, a `ChangeNotifierProxyProvider` will send its value to a `ChangeNotifierProvider`.

### FAQ [#](https://pub.dev/packages/provider#faq)

#### My widget rebuilds too often, what can I do? [#](https://pub.dev/packages/provider#my-widget-rebuilds-too-often-what-can-i-do)

Instead of `Provider.of`, you can use `Consumer`/`Selector`.

Their optional `child` argument allows to rebuild only a very specific part of the widget tree:

```dart
Foo(
  child: Consumer<A>(
    builder: (_, a, child) {
      return Bar(a: a, child: child);
    },
    child: Baz(),
  ),
)
```

In this example, only `Bar` will rebuild when `A` updates. `Foo` and `Baz` won't unnecesseraly rebuild.

To go one step further, it is possible to use `Selector` to ignore changes if they don't have an impact on the widget-tree:

```dart
Selector<List, int>(
  selector: (_, list) => list.length,
  builder: (_, length, __) {
    return Text('$length');
  }
);
```

This snippet will rebuild only if the length of the list changes. But it won't unnecessarily update if an item is updated.

#### Can I obtain two different providers using the same type? [#](https://pub.dev/packages/provider#can-i-obtain-two-different-providers-using-the-same-type)

No. While you can have multiple providers sharing the same type, a widget will be able to obtain only one of them: the closest ancestor.

Instead, you must explicitly give both providers a different type.

Instead of:

```dart
Provider<String>(
  builder: (_) => 'England',
  child: Provider<Sring>(
    builder: (_) => 'London',
    child: ...,
  ),
),
```

Prefer:

```dart
Provider<Country>(
  builder: (_) => Country('England'),
  child: Provider<City>(
    builder: (_) => City('London'),
    child: ...,
  ),
),
```

### `BuilderStateDelegate<T>`

A [StateDelegate](https://pub.dev/documentation/provider/latest/provider/StateDelegate-class.html) that creates and dispose a value from functions.

### `Provider<T>`

A [Provider](https://pub.dev/documentation/provider/latest/provider/Provider-class.html) that manages the lifecycle of the value it provides by delegating to a pair of [ValueBuilder](https://pub.dev/documentation/provider/latest/provider/ValueBuilder.html) and [Disposer](https://pub.dev/documentation/provider/latest/provider/Disposer.html).

It is usually used to avoid making a [StatefulWidget](https://docs.flutter.io/flutter/widgets/StatefulWidget-class.html) for something trivial, such as instantiating a BLoC.

[Provider](https://pub.dev/documentation/provider/latest/provider/Provider-class.html) is the equivalent of a [State.initState](https://docs.flutter.io/flutter/widgets/State/initState.html) combined with [State.dispose](https://docs.flutter.io/flutter/widgets/State/dispose.html). [ValueBuilder](https://pub.dev/documentation/provider/latest/provider/ValueBuilder.html) is called only once in [State.initState](https://docs.flutter.io/flutter/widgets/State/initState.html). We cannot use [InheritedWidget](https://docs.flutter.io/flutter/widgets/InheritedWidget-class.html) as it requires the value to be constructor-initialized and final.

The following example instantiates a `Model` once, and disposes it when [Provider](https://pub.dev/documentation/provider/latest/provider/Provider-class.html) is removed from the tree.

[updateShouldNotify](https://pub.dev/documentation/provider/latest/provider/Provider/updateShouldNotify.html) can optionally be passed to avoid unnecessarily rebuilding dependents when nothing changed. Defaults to `(previous, next) => previous != next`. See [InheritedWidget.updateShouldNotify](https://pub.dev/documentation/provider/latest/provider/Provider/updateShouldNotify.html) for more information.

```dart
class Model {
  void dispose() {}
}

class Stateless extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return Provider<Model>(
      builder: (context) =>  Model(),
      dispose: (context, value) => value.dispose(),
      child: ...,
    );
  }
}
```

## Testing

When testing widgets that consumes providers, it is necessary to add the proper providers in the widget tree above the tested widget.

A typical test may like this:

```dart
final foo = MockFoo();

await tester.pumpWidget(
  Provider<Foo>.value(
    value: foo,
    child: TestedWidget(),
  ),
);
```

Note this example purposefully specified the object type, instead of having it infered. Since we used a mocked class (typically using `mockito`), then we have to downcast the mock to the type of the mocked class. Otherwise, the type inference will resolve to `Provider<MockFoo>` instead of `Provider<Foo>`, which will cause `Provider.of<Foo>` to fail.

### `ChangeNotifierProvider<T extends ChangeNotifier>`

Listens to a [ChangeNotifier](https://docs.flutter.io/flutter/foundation/ChangeNotifier-class.html), expose it to its descendants and rebuilds dependents whenever the [ChangeNotifier.notifyListeners](https://docs.flutter.io/flutter/foundation/ChangeNotifier/notifyListeners.html) is called.

Depending on wether you want to **create** or **reuse** a [ChangeNotifier](https://docs.flutter.io/flutter/foundation/ChangeNotifier-class.html), you will want to use different constructors.

#### Creating a [ChangeNotifier](https://docs.flutter.io/flutter/foundation/ChangeNotifier-class.html):

To create a value, use the default constructor. Creating the instance inside `build` using [ChangeNotifierProvider.value](https://pub.dev/documentation/provider/latest/provider/ChangeNotifierProvider/ChangeNotifierProvider.value.html) will lead to memory leaks and potentially undesired side-effects.

See [this stackoverflow answer](https://stackoverflow.com/questions/52249578/how-to-deal-with-unwanted-widget-build) which explains in further details why using the `.value` constructor to create values is undesired.

DO create a new [ChangeNotifier](https://docs.flutter.io/flutter/foundation/ChangeNotifier-class.html) inside `builder`.

```dart
ChangeNotifierProvider(
  builder: (_) => new MyChangeNotifier(),
  child: ...
)
```

DON'T use [ChangeNotifierProvider.value](https://pub.dev/documentation/provider/latest/provider/ChangeNotifierProvider/ChangeNotifierProvider.value.html) to create your [ChangeNotifier](https://docs.flutter.io/flutter/foundation/ChangeNotifier-class.html).

```dart
ChangeNotifierProvider.value(
  value: new MyChangeNotifier(),
  child: ...
)
```

DON'T create your [ChangeNotifier](https://docs.flutter.io/flutter/foundation/ChangeNotifier-class.html) from variables that can change over the time.

In such situation, your [ChangeNotifier](https://docs.flutter.io/flutter/foundation/ChangeNotifier-class.html) would never be updated when the value changes.

```dart
int count;

ChangeNotifierProvider(
  builder: (_) => new MyChangeNotifier(count),
  child: ...
)
```

If your updating variable comes from a provider, consider using [ChangeNotifierProxyProvider](https://pub.dev/documentation/provider/latest/provider/ChangeNotifierProxyProvider-class.html).
Otherwise, consider making a [StatefulWidget](https://docs.flutter.io/flutter/widgets/StatefulWidget-class.html) and managing your [ChangeNotifier](https://docs.flutter.io/flutter/foundation/ChangeNotifier-class.html) manually.

#### Reusing an existing instance of [ChangeNotifier](https://docs.flutter.io/flutter/foundation/ChangeNotifier-class.html):

If you already have an instance of [ChangeNotifier](https://docs.flutter.io/flutter/foundation/ChangeNotifier-class.html) and want to expose it, you should use [ChangeNotifierProvider.value](https://pub.dev/documentation/provider/latest/provider/ChangeNotifierProvider/ChangeNotifierProvider.value.html) instead of the default constructor.

Failing to do so may dispose the [ChangeNotifier](https://docs.flutter.io/flutter/foundation/ChangeNotifier-class.html) when it is still in use.

- DO use [ChangeNotifierProvider.value](https://pub.dev/documentation/provider/latest/provider/ChangeNotifierProvider/ChangeNotifierProvider.value.html) to provide an existing [ChangeNotifier](https://docs.flutter.io/flutter/foundation/ChangeNotifier-class.html).

```dart
MyChangeNotifier variable;

ChangeNotifierProvider.value(
  value: variable,
  child: ...
)
```

- DON'T reuse an existing [ChangeNotifier](https://docs.flutter.io/flutter/foundation/ChangeNotifier-class.html) using the default constructor.

```dart
MyChangeNotifier variable;

ChangeNotifierProvider(
  builder: (_) => variable,
  child: ...
)
```

### `ChangeNotifierProxyProvider<T, R extends ChangeNotifier>`

A [ChangeNotifierProvider](https://pub.dev/documentation/provider/latest/provider/ChangeNotifierProvider-class.html) that builds and synchronizes a [ChangeNotifier](https://docs.flutter.io/flutter/foundation/ChangeNotifier-class.html) from values obtained from other providers.

To understand better this variation of [ChangeNotifierProvider](https://pub.dev/documentation/provider/latest/provider/ChangeNotifierProvider-class.html), we can look into the following code using the original provider:

```dart
ChangeNotifierProvider(
  builder: (context) {
    return MyChangeNotifier(
      foo: Provider.of<Foo>(context, listen: false),
    );
  },
  child: ...
)
```

In example, we built a `MyChangeNotifier` from a value coming from another provider: `Foo`.

This works as long as `Foo` never changes. But if it somehow updates, then our [ChangeNotifier](https://docs.flutter.io/flutter/foundation/ChangeNotifier-class.html) will never update accordingly.

To solve this issue, we could instead use this class, like so:

```dart
ChangeNotifierProxyProvider<Foo, MyChangeNotifier>(
  initialBuilder: (_) => MyChangeNotifier(),
  builder: (_, foo, myNotifier) => myNotifier
    ..foo = foo,
  child: ...
);
```

In that situation, if `Foo` were to update, then `MyChangeNotifier` will be able to update accordingly.

Notice how `MyChangeNotifier` doesn't receive `Foo` in its constructor anymore. It is now passed through a custom setter instead.

A typical implementation of such `MyChangeNotifier` would be:

```dart
class MyChangeNotifier with ChangeNotifier {
  Foo _foo;
  set foo(Foo value) {
    if (_foo != value) {
      _foo = value;
      // do some extra work, that may call `notifyListeners()`
    }
  }
}
```

- DON'T create the [ChangeNotifier](https://docs.flutter.io/flutter/foundation/ChangeNotifier-class.html) inside `builder` directly.

  This will cause your state to be lost when one of the values used updates. It will also cause uncesserary overhead because it will dispose the previous notifier, then subscribes to the new one.

Instead use properties with custom setters like shown previously, or methods.

```dart
ChangeNotifierProxyProvider<Foo, MyChangeNotifier>(
  // may cause the state to be destroyed unvoluntarily
  builder: (_, foo, myNotifier) => MyChangeNotifier(foo: foo),
  child: ...
);
```

- PREFER using [ProxyProvider](https://pub.dev/documentation/provider/latest/provider/ProxyProvider-class.html) when possible.

  If the created object is only a combination of other objects, without http calls or similar side-effects, then it is likely that an immutable object built using [ProxyProvider](https://pub.dev/documentation/provider/latest/provider/ProxyProvider-class.html) will work.

```dart
R ProxyProviderBuilder (BuildContext context, T value, R previous)
```

### `Consumer<T>`

Obtain [Provider](https://pub.dev/documentation/provider/latest/provider/Provider-class.html) from its ancestors and pass its value to [builder](https://pub.dev/documentation/provider/latest/provider/Consumer/builder.html).

The widget [Consumer](https://pub.dev/documentation/provider/latest/provider/Consumer-class.html) doesn't do any fancy work. It just calls [Provider.of](https://pub.dev/documentation/provider/latest/provider/Provider/of.html) in a new widget, and delegate its `build` implementation to [builder](https://pub.dev/documentation/provider/latest/provider/Consumer/builder.html).

[builder](https://pub.dev/documentation/provider/latest/provider/Consumer/builder.html) must not be null and may be called multiple times (such as when provided value change).

The wiget [Consumer](https://pub.dev/documentation/provider/latest/provider/Consumer-class.html) has two main purposes:

- It allows obtaining a value from a provider when we don't have a [BuildContext](https://docs.flutter.io/flutter/widgets/BuildContext-class.html) that is a descendant of the said provider, and therefore cannot use [Provider.of](https://pub.dev/documentation/provider/latest/provider/Provider/of.html).

Such scenario typically happens when the widget that creates the provider is also one of its consumers, like in the following example:

```dart
@override
Widget build(BuildContext context) {
  return ChangeNotifierProvider(
    builder: (_) => Foo(),
    child: Text(Provider.of<Foo>(context).value),
  );
}
```

This example will throw a [ProviderNotFoundError](https://pub.dev/documentation/provider/latest/provider/ProviderNotFoundError-class.html), because [Provider.of](https://pub.dev/documentation/provider/latest/provider/Provider/of.html) is called with a [BuildContext](https://docs.flutter.io/flutter/widgets/BuildContext-class.html) that is an ancestor of the provider.

Instead, we can use the [Consumer](https://pub.dev/documentation/provider/latest/provider/Consumer-class.html) widget, that will call [Provider.of](https://pub.dev/documentation/provider/latest/provider/Provider/of.html) with its own [BuildContext](https://docs.flutter.io/flutter/widgets/BuildContext-class.html).

Using [Consumer](https://pub.dev/documentation/provider/latest/provider/Consumer-class.html), the previous example will become:

```dart
@override
Widget build(BuildContext context) {
  return ChangeNotifierProvider(
    builder: (_) => Foo(),
    child: Consumer<Foo>(
      builder: (_, foo, __) => Text(foo.value),
    },
  );
}
```

This won't throw a [ProviderNotFoundError](https://pub.dev/documentation/provider/latest/provider/ProviderNotFoundError-class.html) and will correctly build the [Text](https://docs.flutter.io/flutter/widgets/Text-class.html). It will also update the [Text](https://docs.flutter.io/flutter/widgets/Text-class.html) whenever the value `foo` changes.

- It helps with performance optimisation by having more granular rebuilds.

Unless `listen: false` is passed to [Provider.of](https://pub.dev/documentation/provider/latest/provider/Provider/of.html), it means that the widget associated to the [BuildContext](https://docs.flutter.io/flutter/widgets/BuildContext-class.html) passed to [Provider.of](https://pub.dev/documentation/provider/latest/provider/Provider/of.html) will rebuild whenever the obtained value changes.

This is the expected behavior, but sometimes it may rebuild more widgets than needed.

Here's an example:

```dart
 @override
 Widget build(BuildContext context) {
   return FooWidget(
     child: BarWidget(
       bar: Provider.of<Bar>(context),
     ),
   );
 }
```

In such code, only `BarWidget` depends on the value returned by [Provider.of](https://pub.dev/documentation/provider/latest/provider/Provider/of.html). But when `Bar` changes, then both `BarWidget` *and* `FooWidget` will rebuild.

Ideally, only `BarWidget` should be rebuilt. And to achieve that, one solution is to use [Consumer](https://pub.dev/documentation/provider/latest/provider/Consumer-class.html).

To do so, we will wrap *only* the widgets that depends on a provider into a [Consumer](https://pub.dev/documentation/provider/latest/provider/Consumer-class.html):

```dart
 @override
 Widget build(BuildContext context) {
   return FooWidget(
     child: Consumer<Bar>(
       builder: (_, bar, __) => BarWidget(bar: bar),
     ),
   );
 }
```

In this situation, if `Bar` were to update, only `BarWidget` would rebuild.

But what if it was `FooWidget` that depended on a provider? Example:

```dart
 @override
 Widget build(BuildContext context) {
   return FooWidget(
     foo: Provider.of<Foo>(context),
     child: BarWidget(),
   );
 }
```

Using [Consumer](https://pub.dev/documentation/provider/latest/provider/Consumer-class.html), we can handle this kind of scenario using the optional [child](https://pub.dev/documentation/provider/latest/provider/Consumer/child.html) argument:

```dart
 @override
 Widget build(BuildContext context) {
   return Consumer<Foo>(
     builder: (_, foo, child) => FooWidget(foo: foo, child: child),
     child: BarWidget(),
   );
 }
```

In that example, `BarWidget` is built outside of [builder](https://pub.dev/documentation/provider/latest/provider/Consumer/builder.html). Then, the `BarWidget` instance is passed to [builder](https://pub.dev/documentation/provider/latest/provider/Consumer/builder.html) as the last parameter.

This means that when [builder](https://pub.dev/documentation/provider/latest/provider/Consumer/builder.html) is called again with new values, new instance of `BarWidget` will not be recreated. This let Flutter knows that it doesn't have to rebuild `BarWidget`. Therefore in such configuration, only `FooWidget` will rebuild if `Foo` changes.

#### Note:

The widget [Consumer](https://pub.dev/documentation/provider/latest/provider/Consumer-class.html) can also be used inside [MultiProvider](https://pub.dev/documentation/provider/latest/provider/MultiProvider-class.html). To do so, it must returns the `child` passed to [builder](https://pub.dev/documentation/provider/latest/provider/Consumer/builder.html) in the widget tree it creates.

```dart
MultiProvider(
  providers: [
    Provider(builder: (_) => Foo()),
    Consumer<Foo>(
      builder: (context, foo, child) =>
        Provider.value(value: foo.bar, child: child),
    )
  ],
);
```

### DelegateWidget class 

A [StatefulWidget](https://docs.flutter.io/flutter/widgets/StatefulWidget-class.html) that delegates its [State](https://docs.flutter.io/flutter/widgets/State-class.html) implementation to a [StateDelegate](https://pub.dev/documentation/provider/latest/provider/StateDelegate-class.html).

This is useful for widgets that must switch between different [State](https://docs.flutter.io/flutter/widgets/State-class.html) implementation under the same `runtimeType`.

A typical use-case is a non-leaf widget with constructors that behaves differently, as it is necessary for all of its constructors to share the same `runtimeType` or else its descendants would lose their state.

### `FutureProvider<T>`

Listens to a `Future<T>` and exposes `T` to its descendants.

It is considered an error to pass a future that can emit errors without providing a [catchError](https://pub.dev/documentation/provider/latest/provider/FutureProvider/catchError.html) method.

[updateShouldNotify](https://pub.dev/documentation/provider/latest/provider/FutureProvider/updateShouldNotify.html) can optionally be passed to avoid unnecessarily rebuilding dependents when nothing changed. Defaults to `(previous, next) => previous != next`. See [InheritedWidget.updateShouldNotify](https://pub.dev/documentation/provider/latest/provider/FutureProvider/updateShouldNotify.html) for more information.

### `InheritedProvider<T>` 

A generic implementation of an [InheritedWidget](https://docs.flutter.io/flutter/widgets/InheritedWidget-class.html).

Any descendant of this widget can obtain `value` using [Provider.of](https://pub.dev/documentation/provider/latest/provider/Provider/of.html).

Do not use this class directly unless you are creating a custom "Provider". Instead use [Provider](https://pub.dev/documentation/provider/latest/provider/Provider-class.html) class, which wraps [InheritedProvider](https://pub.dev/documentation/provider/latest/provider/InheritedProvider-class.html).

### `ListenableProvider<T extends Listenable>`

Listens to a [Listenable](https://docs.flutter.io/flutter/foundation/Listenable-class.html), expose it to its descendants and rebuilds dependents whenever the listener emits an event.

For usage informations, see [ChangeNotifierProvider](https://pub.dev/documentation/provider/latest/provider/ChangeNotifierProvider-class.html), a subclass of [ListenableProvider](https://pub.dev/documentation/provider/latest/provider/ListenableProvider-class.html) made for [ChangeNotifier](https://docs.flutter.io/flutter/foundation/ChangeNotifier-class.html).

You will generaly want to use [ChangeNotifierProvider](https://pub.dev/documentation/provider/latest/provider/ChangeNotifierProvider-class.html) instead. But [ListenableProvider](https://pub.dev/documentation/provider/latest/provider/ListenableProvider-class.html) is available in case you want to implement [Listenable](https://docs.flutter.io/flutter/foundation/Listenable-class.html) yourself, or use [Animation](https://docs.flutter.io/flutter/animation/Animation-class.html).

### `ListenableProxyProvider<T, R extends Listenable>` 

A variation of [ListenableProvider](https://pub.dev/documentation/provider/latest/provider/ListenableProvider-class.html) that builds its value from values obtained from other providers.

See the discussion on [ChangeNotifierProxyProvider](https://pub.dev/documentation/provider/latest/provider/ChangeNotifierProxyProvider-class.html) for a complete explanation on how to use it.

[ChangeNotifierProxyProvider](https://pub.dev/documentation/provider/latest/provider/ChangeNotifierProxyProvider-class.html) extends [ListenableProxyProvider](https://pub.dev/documentation/provider/latest/provider/ListenableProxyProvider-class.html) to make it work with [ChangeNotifier](https://docs.flutter.io/flutter/foundation/ChangeNotifier-class.html), but the behavior stays the same. Most of the time you'll want to use [ChangeNotifierProxyProvider](https://pub.dev/documentation/provider/latest/provider/ChangeNotifierProxyProvider-class.html) instead. But [ListenableProxyProvider](https://pub.dev/documentation/provider/latest/provider/ListenableProxyProvider-class.html) is exposed in case one wants to use a [Listenable](https://docs.flutter.io/flutter/foundation/Listenable-class.html) implementation other than [ChangeNotifier](https://docs.flutter.io/flutter/foundation/ChangeNotifier-class.html), such as [Animation](https://docs.flutter.io/flutter/animation/Animation-class.html).

### `MultiProvider` 

A provider that merges multiple providers into a single linear widget tree. It is used to improve readability and reduce boilerplate code of having to nest multiple layers of providers.

As such, we're going from:

```dart
Provider<Foo>.value(
  value: foo,
  child: Provider<Bar>.value(
    value: bar,
    child: Provider<Baz>.value(
      value: baz,
      child: someWidget,
    )
  )
)
```

To:

```dart
MultiProvider(
  providers: [
    Provider<Foo>.value(value: foo),
    Provider<Bar>.value(value: bar),
    Provider<Baz>.value(value: baz),
  ],
  child: someWidget,
)
```

The widget tree representation of the two approaches are identical.

### `Provider<T>` 

A [Provider](https://pub.dev/documentation/provider/latest/provider/Provider-class.html) that manages the lifecycle of the value it provides by delegating to a pair of [ValueBuilder](https://pub.dev/documentation/provider/latest/provider/ValueBuilder.html) and [Disposer](https://pub.dev/documentation/provider/latest/provider/Disposer.html).

It is usually used to avoid making a [StatefulWidget](https://docs.flutter.io/flutter/widgets/StatefulWidget-class.html) for something trivial, such as instantiating a BLoC.

[Provider](https://pub.dev/documentation/provider/latest/provider/Provider-class.html) is the equivalent of a [State.initState](https://docs.flutter.io/flutter/widgets/State/initState.html) combined with [State.dispose](https://docs.flutter.io/flutter/widgets/State/dispose.html). [ValueBuilder](https://pub.dev/documentation/provider/latest/provider/ValueBuilder.html) is called only once in [State.initState](https://docs.flutter.io/flutter/widgets/State/initState.html). We cannot use [InheritedWidget](https://docs.flutter.io/flutter/widgets/InheritedWidget-class.html) as it requires the value to be constructor-initialized and final.

The following example instantiates a `Model` once, and disposes it when [Provider](https://pub.dev/documentation/provider/latest/provider/Provider-class.html) is removed from the tree.

[updateShouldNotify](https://pub.dev/documentation/provider/latest/provider/Provider/updateShouldNotify.html) can optionally be passed to avoid unnecessarily rebuilding dependents when nothing changed. Defaults to `(previous, next) => previous != next`. See [InheritedWidget.updateShouldNotify](https://pub.dev/documentation/provider/latest/provider/Provider/updateShouldNotify.html) for more information.

```dart
class Model {
  void dispose() {}
}

class Stateless extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return Provider<Model>(
      builder: (context) =>  Model(),
      dispose: (context, value) => value.dispose(),
      child: ...,
    );
  }
}
```

#### Testing

When testing widgets that consumes providers, it is necessary to add the proper providers in the widget tree above the tested widget.

A typical test may like this:

```dart
final foo = MockFoo();

await tester.pumpWidget(
  Provider<Foo>.value(
    value: foo,
    child: TestedWidget(),
  ),
);
```

Note this example purposefully specified the object type, instead of having it infered. Since we used a mocked class (typically using `mockito`), then we have to downcast the mock to the type of the mocked class. Otherwise, the type inference will resolve to `Provider<MockFoo>` instead of `Provider<Foo>`, which will cause `Provider.of<Foo>` to fail.

### `ProxyProvider<T, R>`

A provider that builds a value based on other providers.

The exposed value is built through [builder](https://pub.dev/documentation/provider/latest/provider/ProxyProvider/builder.html), and then passed to [InheritedProvider](https://pub.dev/documentation/provider/latest/provider/InheritedProvider-class.html).

As opposed to the `builder` parameter of [Provider](https://pub.dev/documentation/provider/latest/provider/Provider-class.html), [builder](https://pub.dev/documentation/provider/latest/provider/ProxyProvider/builder.html) may be called more than once. It will be called once when the widget is mounted, then once whenever any of the [InheritedWidget](https://docs.flutter.io/flutter/widgets/InheritedWidget-class.html) which [ProxyProvider](https://pub.dev/documentation/provider/latest/provider/ProxyProvider-class.html) depends emits an update.

[ProxyProvider](https://pub.dev/documentation/provider/latest/provider/ProxyProvider-class.html) comes in different variants such as [ProxyProvider2](https://pub.dev/documentation/provider/latest/provider/ProxyProvider2-class.html).  This only changes the [builder](https://pub.dev/documentation/provider/latest/provider/ProxyProvider/builder.html) function, such that it takes a different number of arguments.  The `2` in [ProxyProvider2](https://pub.dev/documentation/provider/latest/provider/ProxyProvider2-class.html) means that [builder](https://pub.dev/documentation/provider/latest/provider/ProxyProvider/builder.html) builds its value from **2** other providers.

All variations of [builder](https://pub.dev/documentation/provider/latest/provider/ProxyProvider/builder.html) will receive the [BuildContext](https://docs.flutter.io/flutter/widgets/BuildContext-class.html) as first parameter, and the previously built value as last parameter.

This previously built value will be `null` by default, unless `initialBuilder` is specified – in which case, it will be the value returned by `initialBuilder`.

[builder](https://pub.dev/documentation/provider/latest/provider/ProxyProvider/builder.html) must not be `null`.

#### Note:

While [ProxyProvider](https://pub.dev/documentation/provider/latest/provider/ProxyProvider-class.html) has built-in support with [Provider.of](https://pub.dev/documentation/provider/latest/provider/Provider/of.html), it works with *any* [InheritedWidget](https://docs.flutter.io/flutter/widgets/InheritedWidget-class.html).

As such, `ProxyProvider2<Foo, Bar, Baz>` is just syntax sugar for:

```dart
ProxyProvider<Foo, Baz>(
  builder: (context, foo, baz) {
    final bar = Provider.of<Bar>(context);
  },
  child: ...,
)
```

And it will also work with other `.of` patterns, including [Scrollable.of](https://docs.flutter.io/flutter/widgets/Scrollable/of.html), [MediaQuery.of](https://docs.flutter.io/flutter/widgets/MediaQuery/of.html), and many more:

```dart
ProxyProvider<Foo, Baz>(
  builder: (context, foo, _) {
    final mediaQuery = MediaQuery.of(context);
    return Baz(mediaQuery.size);
  },
  child: ...,
)
```

This previous example will correctly rebuild `Baz` when the `MediaQuery` updates.

#### Constructor

```dart
 ProxyProvider({
     Key key,
     ValueBuilder<R> initialBuilder,
     @required ProxyProviderBuilder<T, R> builder,
     UpdateShouldNotify<R> updateShouldNotify,
     Disposer<R> dispose,
     Widget child
 }) 
```

### `ProxyProviderBuilder<T, R>`

```dart
R ProxyProviderBuilder(
	BuildContext context,
    T value,
    R previous
) 
```

### `ProxyProviderElement` 

An [Element](https://docs.flutter.io/flutter/widgets/Element-class.html) that uses a [ProxyProviderWidget](https://pub.dev/documentation/provider/latest/provider/ProxyProviderWidget-class.html) as its configuration.

### `ProxyProviderState<T extends ProxyProviderWidget>`

A [State](https://docs.flutter.io/flutter/widgets/State-class.html) with an added life-cycle: [didUpdateDependencies](https://pub.dev/documentation/provider/latest/provider/ProxyProviderState/didUpdateDependencies.html).

Widgets such as [ProxyProvider](https://pub.dev/documentation/provider/latest/provider/ProxyProvider-class.html) are expected to build their value from within [didUpdateDependencies](https://pub.dev/documentation/provider/latest/provider/ProxyProviderState/didUpdateDependencies.html) instead of [didChangeDependencies](https://docs.flutter.io/flutter/widgets/State/didChangeDependencies.html).

### `ProxyProviderWidget`

A [StatefulWidget](https://docs.flutter.io/flutter/widgets/StatefulWidget-class.html) that uses [ProxyProviderState](https://pub.dev/documentation/provider/latest/provider/ProxyProviderState-class.html) as [State](https://docs.flutter.io/flutter/widgets/State-class.html).

### `Selector<A, S>` 

An equivalent to [Consumer](https://pub.dev/documentation/provider/latest/provider/Consumer-class.html) that can filter updates by selecting a limited amount of values and prevent rebuild if they don't change.

[Selector](https://pub.dev/documentation/provider/latest/provider/Selector-class.html) will obtain a value using [Provider.of](https://pub.dev/documentation/provider/latest/provider/Provider/of.html), then pass that value to `selector`. That `selector` callback is then tasked to return an object that contains only the informations needed for `builder` to complete.

The object returned by `selector` should be immutable and override `operator==` such that two objects with the same content are equal, even if they are not `identical`.

As such, to select multiple values, the easiest solution is to use a "Tuple" from [tuple](https://pub.dev/packages/tuple):

```dart
Selector<Foo, Tuple2<Bar, Baz>>(
  selector: (_, foo) => Tuple2(foo.bar, foo.baz),
  builder: (_, data, __) {
    return Text('${data.item1}  ${data.item2}');
  }
)
```

In that example, `builder` will be called again only if `foo.bar` or `foo.baz` changes.

For generic usage informations, see [Consumer](https://pub.dev/documentation/provider/latest/provider/Consumer-class.html).

### `SingleChildCloneableWidget` 

A base class for providers so that [MultiProvider](https://pub.dev/documentation/provider/latest/provider/MultiProvider-class.html) can regroup them into a linear list.

### `SingleValueDelegate<T>` 

Stores an immutable value.

### `StreamProvider<T>` 

Listens to a `Stream<T>` and exposes `T` to its descendants.

Its main use-case is to provide to a large number of a widget the content of a `Stream`, without caring about reacting to events.

A typical example would be to expose the battery level, or a Firebase query. Trying to use `Stream` to replace [ChangeNotifier](https://docs.flutter.io/flutter/foundation/ChangeNotifier-class.html) is outside of the scope of this class.

It is considered an error to pass a stream that can emit errors without providing a [catchError](https://pub.dev/documentation/provider/latest/provider/StreamProvider/catchError.html) method.

[initialData](https://pub.dev/documentation/provider/latest/provider/StreamProvider/initialData.html) determines the value exposed until the `Stream` emits a value. If omitted, defaults to `null`. [updateShouldNotify](https://pub.dev/documentation/provider/latest/provider/StreamProvider/updateShouldNotify.html) can optionally be passed to avoid unnecessarily rebuilding dependents when nothing changed. Defaults to `(previous, next) => previous != next`. See [InheritedWidget.updateShouldNotify](https://pub.dev/documentation/provider/latest/provider/StreamProvider/updateShouldNotify.html) for more information.

### `ValueDelegateWidget<T>` 

A [DelegateWidget](https://pub.dev/documentation/provider/latest/provider/DelegateWidget-class.html) that accepts only [ValueStateDelegate](https://pub.dev/documentation/provider/latest/provider/ValueStateDelegate-class.html) as [delegate](https://pub.dev/documentation/provider/latest/provider/ValueDelegateWidget/delegate.html).

### `ValueListenableProvider<T>`

Listens to a [ValueListenable](https://docs.flutter.io/flutter/foundation/ValueListenable-class.html) and expose its current value.

### `ValueStateDelegate<T>`

A base class for [StateDelegate](https://pub.dev/documentation/provider/latest/provider/StateDelegate-class.html) that exposes a [value](https://pub.dev/documentation/provider/latest/provider/ValueStateDelegate/value.html) of type `T`.

