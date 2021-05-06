import { Nested } from '../out';

it('success with valid object', () => {
  expect(() =>
    Nested.validate({
      data: {
        data: 1,
      },
    }),
  ).not.toThrow();
});

it('success when nested object contains extra fields', () => {
  expect(() =>
    Nested.validate({
      data: {
        data: 1,
        extra: 'extra',
      },
    }),
  ).not.toThrow();
});

it('fails when nested field has wrong type', () => {
  expect(() =>
    Nested.validate({
      data: {
        data: '1',
      },
    }),
  ).toThrow();
});

it('fails when outer field has wrong type', () => {
  expect(() =>
    Nested.validate({
      data: 1,
    }),
  ).toThrow();
});
