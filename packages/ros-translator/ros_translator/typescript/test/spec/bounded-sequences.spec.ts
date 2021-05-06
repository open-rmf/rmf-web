import { BoundedSequence } from '../out';

it('success with length === capacity', () => {
  expect(() =>
    BoundedSequence.validate({
      data: [1, 2, 3],
    }),
  ).not.toThrow();
});

it('success with field < capacity', () => {
  expect(() =>
    BoundedSequence.validate({
      data: [1, 2],
    }),
  ).not.toThrow();
});

it('fails with missing field', () => {
  expect(() => BoundedSequence.validate({})).toThrow();
});

it('fails with length > capacity', () => {
  expect(() =>
    BoundedSequence.validate({
      data: [1, 2, 3, 4],
    }),
  ).toThrow();
});
