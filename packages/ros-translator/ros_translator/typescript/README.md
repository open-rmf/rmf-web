Some example usage with typescript

```ts
import { Time } from './out'

// validating a message
try {
  Time.validate({ sec: 0, nanosec: 0 });
} catch (e) {
  console.error(e);
}

// type hints when creating a message
const timeMsg: Time = { sec: 0, nanosec: 0 };

// defining functions that take a ros message
function foo(time: Time): void { }
```

The mapping between ROS types are:

```python
PRIMITIVE_TYPES = {
  'bool': 'boolean',
  'byte': 'number',
  'char': 'string',
  'float32': 'number',
  'float64': 'number',
  'int8': 'number',
  'int16': 'number',
  'int32': 'number',
  'int64': 'number',
  'string': 'string',
  'wstring': 'string',
  'uint8': 'number',
  'uint16': 'number',
  'uint32': 'number',
  'uint64': 'number',
}

TYPED_ARRAY_TYPES = {
  'byte': 'Uint8Array',
  'float32': 'Float32Array',
  'float64': 'Float64Array',
  'int8': 'Int8Array',
  'int16': 'Int16Array',
  'int32': 'Int32Array',
  'int64': 'BigInt64Array',
  'uint8': 'Uint8Array',
  'uint16': 'Uint16Array',
  'uint32': 'Uint32Array',
  'uint64': 'BigUint64Array',
}
```
