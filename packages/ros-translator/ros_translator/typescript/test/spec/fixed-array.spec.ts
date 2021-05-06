import { FixedArray } from '../out';

it('success with valid object', () => {
  expect(() =>
    FixedArray.validate({
      data: [1, 2, 3],
    }),
  ).not.toThrow();
});

it('success with valid object with extra fields', () => {
  expect(() =>
    FixedArray.validate({
      data: [1, 2, 3],
      more: 'extra',
    }),
  ).not.toThrow();
});

it('fails with missing field', () => {
  expect(() => FixedArray.validate({})).toThrow();
});

it('fails with wrong array size', () => {
  expect(() =>
    FixedArray.validate({
      data: [1, 2],
    }),
  ).toThrow();
});
