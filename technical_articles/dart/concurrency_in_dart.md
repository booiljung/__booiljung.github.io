# Concurrency in Dart

Dart supports concurrent programming with async-await, isolates, and classes such as `Future` and `Stream`. This page gives an overview of async-await, `Future`, and `Stream`, but it’s mostly about isolates.

Within an app, all Dart code runs in an *isolate.* Each Dart isolate has a single thread of execution and shares no mutable objects with other isolates. To communicate with each other, isolates use message passing. Although Dart’s isolate model is built with underlying primitives such as processes and threads that the operating system provides, the Dart VM’s use of these primitives is an implementation detail that this page doesn’t discuss.

Many Dart apps use only one isolate (the *main isolate*), but you can create additional isolates, enabling parallel code execution on multiple processor cores.



  **Platform note:**  All apps can use async-await, `Future`, and `Stream`.  Isolates are implemented only on the [Dart Native platform](https://dart.dev/overview#platform);  Dart web apps can use [web workers](https://developer.mozilla.org/en-US/docs/Web/API/Web_Workers_API/Using_web_workers) for similar functionality.

## Asynchrony types and syntax

If you’re already familiar with `Future`, `Stream`, and async-await, then you can skip ahead to the [isolates section](https://dart.dev/guides/language/concurrency#how-isolates-work).

### Future and Stream types

The Dart language and libraries use `Future` and `Stream` objects to represent values to be provided in the future. For example, a promise to eventually provide an `int` value is typed as `Future<int>`. A promise to provide a series of `int` values has the type `Stream<int>`.

As another example, consider the dart:io methods for reading files. The synchronous `File` method [`readAsStringSync()`](https://api.dart.dev/stable/dart-io/File/readAsStringSync.html) reads a file synchronously, blocking until the file is either fully read or an error occurs. The method then either returns an object of type `String` or throws an exception. The asynchronous equivalent, [`readAsString()`](https://api.dart.dev/stable/dart-io/File/readAsString.html), immediately returns an object of type `Future<String>`. At some point in the future, the `Future<String>` completes with either a string value or an error.

#### Why asynchronous code matters

It matters whether a method is synchronous or asynchronous because most apps need to do more than one thing at a time.

Asynchronous computations are often the result of performing computations outside of the current Dart code;  this includes computations that don’t complete immediately,  and where you aren’t willing to block your Dart code waiting for the result. For example, an app might start an HTTP request, but need to update its display or respond to user input before the HTTP request completes. Asynchronous code helps apps stay responsive.

These scenarios include operating system calls like non-blocking I/O, performing an HTTP request, or communicating with a browser.  Other scenarios include waiting for computations performed in another Dart isolate as described below,  or maybe just waiting for a timer to trigger.  All of these processes either run in a different thread,  or are handled by the operating system or the Dart runtime,  which allows Dart code to run concurrently with the computation.

### The async-await syntax

The `async` and `await` keywords provide a declarative way to define asynchronous functions and use their results.

Here’s an example of some synchronous code that blocks while waiting for file I/O:

```dart
void main() {
  // Read some data.
  final fileData = _readFileSync();
  final jsonData = jsonDecode(fileData);

  // Use that data.
  print('Number of JSON keys: ${jsonData.length}');
}

String _readFileSync() {
  final file = File(filename);
  final contents = file.readAsStringSync();
  return contents.trim();
}
```

Here’s similar code, but with changes (highlighted) to make it asynchronous:

```dart
void main() async {
  // Read some data.
  final fileData = await _readFileAsync();
  final jsonData = jsonDecode(fileData);

  // Use that data.
  print('Number of JSON keys: ${jsonData.length}');
}

Future<String> _readFileAsync() async {
  final file = File(filename);
  final contents = await file.readAsString();
  return contents.trim();
}
```

The `main()` function uses the `await` keyword in front of `_readFileAsync()` to let other Dart code (such as event handlers) use the CPU while native code (file I/O) executes. Using `await` also has the effect of converting the `Future<String>` returned by `_readFileAsync()` into a `String`. As a result, the `contents` variable has the implicit type `String`.



 **Note:**  The `await` keyword works only in functions that  have `async` before the function body.

As the following figure shows, the Dart code pauses while `readAsString()` executes non-Dart code, in either the Dart virtual machine (VM) or the operating system (OS). Once `readAsString()` returns a value, Dart code execution resumes.

![Flowchart-like figure showing app code executing from start to exit, waiting for native I/O in between](https://dart.dev/guides/language/concurrency/images/basics-await.png)

If you’d like to learn more about using `async`, `await`, and futures, visit the [asynchronous programming codelab](https://dart.dev/codelabs/async-await).

## How isolates work

Most modern devices have multi-core CPUs. To take advantage of all those cores, developers sometimes use shared-memory threads running concurrently. However, shared-state concurrency is [error prone](https://en.wikipedia.org/wiki/Race_condition#In_software) and can lead to complicated code.

Instead of threads, all Dart code runs inside of isolates. Each isolate has its own memory heap, ensuring that none of the state in an isolate is accessible from any other isolate. Because there’s no shared memory, you don’t have to worry about [mutexes or locks](https://en.wikipedia.org/wiki/Lock_(computer_science)).

Using isolates, your Dart code can perform multiple independent tasks at once, using additional processor cores if they’re available. Isolates are like threads or processes, but each isolate has its own memory and a single thread running an event loop.

### The main isolate

You often don’t need to think about isolates at all. A typical Dart app executes all its code in the app’s main isolate, as shown in the following figure:

![A figure showing a main isolate, which runs `main()`, responds to events, and then exits](https://dart.dev/guides/language/concurrency/images/basics-main-isolate.png)

Even single-isolate programs can execute smoothly by using async-await to wait for asynchronous operations to complete before continuing to the next line of code. A well-behaved app starts quickly, getting to the event loop as soon as possible. The app then responds to each queued event promptly, using asynchronous operations as necessary.

### The isolate life cycle

As the following figure shows, every isolate starts by running some Dart code, such as the `main()` function. This Dart code might register some event listeners—to  respond to user input or file I/O, for example. When the isolate’s initial function returns, the isolate stays around if it needs to handle events. After handling the events, the isolate exits.

![A more general figure showing that any isolate runs some code, optionally responds to events, and then exits](https://dart.dev/guides/language/concurrency/images/basics-isolate.png)

### Event handling

In a client app, the main isolate’s event queue might contain repaint requests and notifications of tap and other UI events. For example, the following figure shows a repaint event, followed by a tap event, followed by two repaint events. The event loop takes events from the queue in first in, first out order.

![A figure showing events being fed, one by one, into the event loop](https://dart.dev/guides/language/concurrency/images/event-loop.png)

Event handling happens on the main isolate after `main()` exits. In the following figure, after `main()` exits, the main isolate handles the first repaint event. After that, the main isolate handles the tap event, followed by a repaint event.

![A figure showing the main isolate executing event handlers, one by one](https://dart.dev/guides/language/concurrency/images/event-handling.png)

If a synchronous operation takes too much processing time, the app can become unresponsive. In the following figure, the tap-handling code takes too long, so subsequent events are handled too late. The app might appear to freeze, and any animation it performs might be jerky.

![A figure showing a tap handler with a too-long execution time](https://dart.dev/guides/language/concurrency/images/event-jank.png)

In client apps, the result of a too-lengthy synchronous operation is often [janky (non-smooth) UI animation](https://docs.flutter.dev/perf/rendering-performance). Worse, the UI might become completely unresponsive.

### Background workers

If your app’s UI becomes unresponsive due to  a time-consuming computation—[parsing a large JSON file](https://docs.flutter.dev/cookbook/networking/background-parsing),  for example—consider offloading that computation to a worker isolate, often called a *background worker.* A common case, shown in the following figure, is spawning a simple worker isolate that performs a computation and then exits. The worker isolate returns its result in a message when the worker exits.

![A figure showing a main isolate and a simple worker isolate](https://dart.dev/guides/language/concurrency/images/isolate-bg-worker.png)

Each isolate message can deliver one object, which includes anything that’s transitively reachable from that object. Not all object types are sendable, and the send fails if any transitively reachable object is unsendable. For example, you can send an object of type `List<Object>` only if none of the objects in the list is unsendable. If one of the objects is, say, a `Socket`, then the send fails because sockets are unsendable.

For information on the kinds of objects that you can send in messages, see the API reference documentation for the [`send()` method](https://api.dart.dev/stable/dart-isolate/SendPort/send.html).

A worker isolate can perform I/O (reading and writing files, for example), set timers, and more. It has its own memory and doesn’t share any state with the main isolate. The worker isolate can block without affecting other isolates.

## Code examples

This section discusses some examples that use the `Isolate` API to implement isolates.

![Flutter logo](https://dart.dev/assets/img/shared/flutter/icon/64.png) **Flutter note:**  If you’re using Flutter on a non-web platform,  then instead of using the `Isolate` API directly,  consider using the [Flutter `compute()` function](https://docs.flutter.dev/cookbook/networking/background-parsing#4-move-this-work-to-a-separate-isolate).  The `compute()` function is a simple way to  move a single function call to a worker isolate.

### Implementing a simple worker isolate

This section shows the implementation for a main isolate and the simple worker isolate that it spawns. The worker isolate executes a function and then exits, sending the main isolate a single message as it exits. (The [Flutter `compute()` function](https://docs.flutter.dev/cookbook/networking/background-parsing#4-move-this-work-to-a-separate-isolate) works in a similar way.)

This example uses the following isolate-related API:

- [`Isolate.spawn()`](https://api.dart.dev/stable/dart-isolate/Isolate/spawn.html) and [`Isolate.exit()`](https://api.dart.dev/stable/dart-isolate/Isolate/exit.html)
- [`ReceivePort`](https://api.dart.dev/stable/dart-isolate/ReceivePort-class.html) and [`SendPort`](https://api.dart.dev/stable/dart-isolate/SendPort-class.html)

Here’s the code for the main isolate:

```dart
void main() async {
  // Read some data.
  final jsonData = await _parseInBackground();

  // Use that data
  print('Number of JSON keys: ${jsonData.length}');
}

// Spawns an isolate and waits for the first message
Future<Map<String, dynamic>> _parseInBackground() async {
  final p = ReceivePort();
  await Isolate.spawn(_readAndParseJson, p.sendPort);
  return await p.first as Map<String, dynamic>;
}
```

The `_parseInBackground()` function contains the code that *spawns* (creates and starts) the isolate for the background worker, and then returns the result:

1. Before spawning the isolate, the code creates a `ReceivePort`, which enables the worker isolate to send messages to the main isolate.
2. Next is the call to `Isolate.spawn()`, which creates and starts the isolate for the background worker. The first argument to `Isolate.spawn()` is the function that the worker isolate executes: `_readAndParseJson`. The second argument is the `SendPort` that the worker isolate can use to send messages to the main isolate. The code doesn’t *create* a `SendPort`; it uses the `sendPort` property of the `ReceivePort`.
3. Once the isolate is spawned, the main isolate waits for the result. Because the `ReceivePort` class implements `Stream`, the [`first`](https://api.dart.dev/stable/dart-async/Stream/first.html) property is an easy way to get the single message that the worker isolate sends.

The spawned isolate executes the following code:

```dart
Future<void> _readAndParseJson(SendPort p) async {
  final fileData = await File(filename).readAsString();
  final jsonData = jsonDecode(fileData);
  Isolate.exit(p, jsonData);
}
```

The relevant statement is the last one, which exits the isolate, sending `jsonData` to the passed-in `SendPort`. Message passing using `SendPort.send` normally involves data copying, and thus can be slow. However, when you send the data using `Isolate.exit()`, then the memory that holds the message in the exiting isolate isn’t copied, but instead is transferred to the receiving isolate. The sender will nonetheless perform a verification pass to ensure the objects are allowed to be transferred.



 **Version note:**  `Isolate.exit()` was added in 2.15.  Previous releases support only explicit message passing,  using `Isolate.send()` as shown in the next section’s example.

The following figure illustrates the communication between the main isolate and the worker isolate:

![A figure showing the previous snippets of code running in the main isolate and in the worker isolate](https://dart.dev/guides/language/concurrency/images/isolate-api.png)

### Sending multiple messages between isolates

If you need more communication between isolates, then you need to use the [`send()` method](https://api.dart.dev/stable/dart-isolate/SendPort/send.html) of `SendPort`. One common pattern, which the following figure shows, is for the main isolate to send a request message to the worker isolate, which then sends one or more reply messages.

![A figure showing the main isolate spawning the isolate and then sending a request message, which the worker isolate responds to with a reply message; two request-reply cycles are shown](https://dart.dev/guides/language/concurrency/images/isolate-custom-bg-worker.png)

For examples of sending multiple messages, see the following [isolate samples](https://github.com/dart-lang/samples/tree/master/isolates):

- [send_and_receive.dart](https://github.com/dart-lang/samples/tree/master/isolates/bin/send_and_receive.dart), which shows how to send a message from the main isolate to the spawned isolate. It’s otherwise similar to the preceding example.
- [long_running_isolate.dart](https://github.com/dart-lang/samples/tree/master/isolates/bin/long_running_isolate.dart), which shows how to spawn a long-running isolate that receives and sends multiple times.

## Performance and isolate groups

When an isolate calls [`Isolate.spawn()`](https://api.dart.dev/stable/dart-isolate/Isolate/spawn.html), the two isolates have the same executable code and are in the same *isolate group*. Isolate groups enable performance optimizations such as sharing code; a new isolate immediately runs the code owned by the isolate group. Also, `Isolate.exit()` works only when the isolates are in the same isolate group.

In some special cases, you might need to use [`Isolate.spawnUri()`](https://api.dart.dev/stable/dart-isolate/Isolate/spawnUri.html), which sets up the new isolate with a copy of the code that’s at the specified URI. However, `spawnUri()` is much slower than `spawn()`, and the new isolate isn’t in its spawner’s isolate group. Another performance consequence is that message passing is slower when isolates are in different groups.