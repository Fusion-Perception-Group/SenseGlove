# Results and Errors

Exceptions are supported in ELFE, in fact it is recommended to use exceptions to handle errors. However we understand that in the embedded world, exceptions may be too expensive and unpredictable. Therefore, ELFE provides a dual-mode error reporting mechanism.

## Result<> class

The `Result<>` class is a template class that is used to return the result of a function.
