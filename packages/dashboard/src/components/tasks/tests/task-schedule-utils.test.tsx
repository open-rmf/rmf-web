import React from 'react';
import { toISOStringWithTimezone } from '../task-schedule-utils';

it('converts date to ISO string while retaining timezone', async () => {
  const date_str = '2024-02-14T19:20:34+08:00';
  expect(toISOStringWithTimezone(new Date(date_str))).toBe(date_str);
});

export {};
