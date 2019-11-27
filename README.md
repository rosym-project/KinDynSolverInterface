# KinDynSolverInterface
An library that interfaces with different kinematic and dynamic solvers

This library offers a unified interface for the end-user,
encapsulating the API of the back-end. Such back-end could libraries
such as RBDL, KDL, iDynTree, etc. 

Such approach achieves three goals:
1. It would be possible to entirely switch to another solver without
   changing the user code (e.g., if new solvers with better
   performance appear, or, if a certain library performs better in
   different domains)
2. The preliminary steps are concentrated in one place (for KDL for
   instance, the whole URDFtoKDL business is done outside of the users
   space)
3. Creates some algorithmic refactoring conveniences: e.g., switching
   between recursive Newton Euler to/from composite rigid body can be
   done in one place without changing the user code.

Currently I will do a RBDL version and keep the rest as TODOs.

## Dependencies
### RBDL
Can be obtained from
[https://bitbucket.org/rbdl/rbdl/src](https://bitbucket.org/rbdl/rbdl/src)
(mercurial) or
[https://github.com/erwincoumans/rbdl.git](https://github.com/erwincoumans/rbdl.git)
(git). Then:

``` sh
git clone -b dev https://github.com/erwincoumans/rbdl.git
mkdir build && cd build
cmake .. -DRBDL_BUILD_ADDON_URDFREADER=ON -DCMAKE_INSTALL_PREFIX=/path/to/your/install
make
make install
```

The install path then can be pass to this project's cmake when
configuring the project by
`-DCMAKE_PREFIX_PATH=/path/to/your/install`.

## Notes:
### <a name "rbdl-issue"></a> RBDL
1. RBDL has this method called `UpdateKinematics()` which is a bit
   peculiar: each function call (e.g., `CalcPointJacobian6D`) has an
   optional boolean parameter that if `=true` calls the
   `UpdateKinematics()`. If we pass false, then function
   kinematic/dynamic calls return zero. So we have two options: either
   call the factions with `=true`, or, make a call to
   `UpdateKinematics()` from somewhere like the user space and update
   everything. If the update is not computationally expensive, I
   rather to for the first option. For the moment I pass true.
   
   
## TODO
1. Resolve the [issue of RBDL's `UpdateKinematics()`](rbdl-issue).
