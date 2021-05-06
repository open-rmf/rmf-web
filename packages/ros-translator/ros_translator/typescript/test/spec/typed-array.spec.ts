import { TypedArray } from '../out';

it('success with typed array', () => {
  expect(() =>
    TypedArray.validate({
      data: Int32Array.from([1, 2, 3]),
    }),
  ).not.toThrow();
});

it('success with plain array', () => {
  expect(() =>
    TypedArray.validate({
      data: [1, 2, 3],
    }),
  ).not.toThrow();
});

it('fails with typed array of wrong type', () => {
  expect(() =>
    TypedArray.validate({
      data: Uint32Array.from([1, 2, 3]),
    }),
  ).toThrow();
});

it('fails with wrong type', () => {
  expect(() =>
    TypedArray.validate({
      data: 1,
    }),
  ).toThrow();
});
