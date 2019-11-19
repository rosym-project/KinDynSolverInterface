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

