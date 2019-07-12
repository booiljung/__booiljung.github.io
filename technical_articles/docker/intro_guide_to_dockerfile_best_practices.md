원문: [Intro Guide to Dockerfile Best Practices](https://blog.docker.com/wp-comments-post.php)

# Intro Guide to Dockerfile Best Practices

By [Tibor Vass](https://blog.docker.com/author/tibor-vass/)July 02 2019

​                                                         [docker](https://blog.docker.com/tag/docker/), [Dockerfiles](https://blog.docker.com/tag/dockerfiles/), [Multi-stage Builds](https://blog.docker.com/tag/multi-stage-builds/)            

현재 GitHub에는 [Dockerfiles](https://docs.docker.com/engine/reference/builder/)가 백만 개 이상 있지만 모든 Dockerfiles가 똑같이 생성되는 것은 아닙니다. 효율성은 매우 중요합니다. 이 블로그 시리즈에서는 점증적인 빌드 시간, 이미지 크기, 유지 관리 가능성, 보안 및 반복성과 같은 더 나은 Dockerfile을 작성하는 데 도움이되는 Dockerfile 모범 사례(best practices)의 5 가지 영역을 다룹니다. Docker로 시작한 사람이라면 이 첫 번째 블로그 게시물은 당신을 위한 것입니다! 이 시리즈의 다음 글은 더욱 발전 될 것입니다.

중요 사항: 아래 팁은 메이븐 (Maven)을 기반으로하는 예제 자바 프로젝트의 Dockerfiles가 개선되는 여정을 따라 갑니다. 따라서 마지막 Dockerfile은 권장 Dockerfile이지만 모든 중간 단계 Dockerfile은 모범 사례를 설명하기 위한 용도로만 사용됩니다.

## 증분 빌드 시간 (Incremental build time)

개발 사이클에서 Docker 이미지를 작성하고 코드를 변경 한 다음 다시 작성하면 캐싱을 활용하는 것이 중요합니다. 캐싱은 필요하지 않을 때 빌드 단계를 다시 실행하지 않도록 도와줍니다.

#### Tip #1: 캐싱 순서 (Order matters for caching)

![img](https://i0.wp.com/blog.docker.com/wp-content/uploads/2019/07/ef41db8f-fe5e-4a78-940a-6a929db7929d-1.jpg?ssl=1)

그러나 Dockerfile에서 파일을 변경하거나 행을 수정하여 단계의 캐시를 무효화하면 캐시의 후속 단계가 중단되므로 빌드 단계 (Dockerfile 지침)의 순서가 중요합니다. 캐싱을 최적화하기 위해 가장 자주 변경하는 단계부터 가장 자주 변경하는 단계까지 단계를 순서대로 하세요.

#### Tip #2: 캐시 흉상을 제한하는보다 구체적인 COPY (More specific COPY to limit cache busts)

![img](https://i1.wp.com/blog.docker.com/wp-content/uploads/2019/07/0c1d0c4e-406c-468c-b6ba-b71ac68b9c84.jpg?ssl=1)

Only  copy what’s needed. If possible, avoid “COPY  .” When copying files  into your image, make sure you are very specific about what you want to  copy. Any changes to the files being copied will break the cache. In the  example above, only the pre-built jar application is needed inside the  image, so only copy that. That way unrelated file changes will not  affect the cache.

#### Tip #3: Identify cacheable units such as apt-get update & install

![img](https://i0.wp.com/blog.docker.com/wp-content/uploads/2019/07/2322a39e-bd7e-4a2b-9a8f-548a97dbacb4.jpg?ssl=1)

Each RUN instruction can be seen as a  cacheable unit of execution. Too many of them can be unnecessary, while  chaining all commands into one RUN instruction can bust the cache  easily, hurting the development cycle. When installing packages from  package managers, you always want to update the index and install  packages in the same RUN: they form together one cacheable unit.  Otherwise you risk installing outdated packages.

## Reduce Image size

Image size can be important because smaller images equal faster deployments and a smaller attack surface.

#### Tip #4: Remove unnecessary dependencies

![img](https://i1.wp.com/blog.docker.com/wp-content/uploads/2019/07/a1b36f64-1a30-45bf-8fcd-4f88437c189e.jpg?ssl=1)

Remove unnecessary dependencies and do not  install debugging tools. If needed debugging tools can always be  installed later. Certain package managers such as apt, automatically  install packages that are recommended by the user-specified package,  unnecessarily increasing the footprint. Apt has the  –no-install-recommends flag which ensures that dependencies that were not actually needed are not installed. If they are needed, add them explicitly.

#### Tip #5: Remove package manager cache

![img](https://i1.wp.com/blog.docker.com/wp-content/uploads/2019/07/363961a4-005e-46fc-963b-f7b690be12ef.jpg?ssl=1)

Package  managers maintain their own cache which may end up in the image. One  way to deal with it is to remove the cache in the same RUN instruction  that installed packages. Removing it in another RUN instruction would  not reduce the image size.

There are further ways to reduce image size such as multi-stage builds which will be covered at the end of this blog post. The  next set of best practices will look at how we can optimize for  maintainability, security, and repeatability of the Dockerfile.

## Maintainability

#### Tip #6: Use official images when possible

![img](https://i0.wp.com/blog.docker.com/wp-content/uploads/2019/07/f336014d-d2aa-4c1b-a2bd-e1d5d6ed0d93.jpg?ssl=1)

Official  images can save a lot of time spent on maintenance because all the  installation steps are done and best practices are applied. If you have  multiple projects, they can share those layers because they use exactly  the same base image.

#### Tip #7: Use more specific tags

![img](https://i0.wp.com/blog.docker.com/wp-content/uploads/2019/07/9d991da9-bdb9-4108-8b36-296a5a3772aa.jpg?ssl=1)

Do  not use the latest tag. It has the convenience of always being  available for official images on Docker Hub but there can be breaking  changes over time. Depending on how far apart in time you rebuild the  Dockerfile without cache, you may have failing builds.

Instead, use more specific tags for your  base images. In this case, we’re using openjdk. There are a lot more  tags available so check out the [Docker Hub documentation](https://hub.docker.com/_/openjdk) for that image which lists all the existing variants.

#### Tip #8: Look for minimal flavors

![img](https://i0.wp.com/blog.docker.com/wp-content/uploads/2019/07/6c486200-5198-4457-86c0-b5275e70e699.jpg?ssl=1)

Some of those tags have minimal  flavors which means they are even smaller images. The slim variant is  based on a stripped down Debian, while the alpine variant is based on  the even smaller Alpine Linux distribution image. A notable difference  is that debian still uses GNU libc while alpine uses musl libc which,  although much smaller, may in some cases cause compatibility issues. In  the case of openjdk, the jre flavor only contains the java runtime, not  the sdk; this also drastically reduces the image size.

## Reproducibility

So  far the Dockerfiles above have assumed that your jar artifact was built  on the host. This is not ideal because you lose the benefits of the  consistent environment provided by containers. For instance if your Java  application depends on specific libraries it may introduce unwelcome  inconsistencies depending on which computer the application is built.

#### Tip #9: Build from source in a consistent environment

The source code is the source of truth from which you want to build a Docker image. The Dockerfile is simply the blueprint.

![img](https://i2.wp.com/blog.docker.com/wp-content/uploads/2019/07/f393ad07-c25d-4241-a40f-c6168e0ba4dd.jpg?ssl=1)

You  should start by identifying all that’s needed to build your  application. Our simple Java application requires Maven and the JDK, so  let’s base our Dockerfile off of a specific minimal official maven image  from Docker Hub, that includes the JDK. If you needed to install more  dependencies, you could do so in a RUN step.

The pom.xml and src folders are copied in as they are needed for the final RUN step that produces the app.jar application with `mvn package`. (The -e flag is to show errors and -B to run in non-interactive aka “batch” mode).

We  solved the inconsistent environment problem, but introduced another  one: every time the code is changed, all the dependencies described in  pom.xml are fetched. Hence the next tip.

#### Tip #10: Fetch dependencies in a separate step

![img](https://i0.wp.com/blog.docker.com/wp-content/uploads/2019/07/41ea71ce-11c3-42a3-8d2b-05fe20901745.jpg?ssl=1)

By  again thinking in terms of cacheable units of execution, we can decide  that fetching dependencies is a separate cacheable unit that only needs  to depend on changes to pom.xml and not the source code. The RUN step  between the two COPY steps tells Maven to only fetch the dependencies.

There  is one more problem that got introduced by building in consistent  environments: our image is way bigger than before because it includes  all the build-time dependencies that are not needed at runtime.

#### Tip #11: Use multi-stage builds to remove build dependencies (recommended Dockerfile)

![img](https://i1.wp.com/blog.docker.com/wp-content/uploads/2019/07/97ec1992-f0df-4c8f-82a0-e177c230e5c5.jpg?ssl=1)

Multi-stage  builds are recognizable by the multiple FROM statements. Each FROM  starts a new stage. They can be named with the AS keyword which we use  to name our first stage “builder” to be referenced later. It will  include all our build dependencies in a consistent environment.

The second stage is our final stage which  will result in the final image. It will include the strict necessary for  the runtime, in this case a minimal JRE (Java Runtime) based on Alpine.  The intermediary builder stage will be cached but not present in the  final image. In order to get build artifacts into our final image, use `COPY --from=STAGE_NAME`. In this case, STAGE_NAME is builder.

![img](https://i2.wp.com/blog.docker.com/wp-content/uploads/2019/07/80c3c350-5f7e-4cf1-ab3e-89df755b3c33.jpg?ssl=1)

Multi-stage builds is the go-to solution to remove build-time dependencies.

We went from building bloated images  inconsistently to building minimal images in a consistent environment  while being cache-friendly. In the next blog post, we will dive more into other uses of multi-stage builds.