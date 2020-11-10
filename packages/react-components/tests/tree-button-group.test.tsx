import React from 'react';
import { TreeButtonGroup } from '../lib';
import { render, fireEvent } from '@testing-library/react';

test('It disabled buttons', () => {
  const root = render(
    <TreeButtonGroup disableClear={true} disableReset={true} disableRestore={true} />,
  );
  expect(root.container.querySelector('button#clear-button').hasAttribute('disabled')).toBeTruthy();
  expect(root.container.querySelector('button#reset-button').hasAttribute('disabled')).toBeTruthy();
  expect(
    root.container.querySelector('button#restore-button').hasAttribute('disabled'),
  ).toBeTruthy();
});

test('It executes callbacks correctly', () => {
  const handleResetClick = jest.fn();
  const handleClearClick = jest.fn();
  const handleRestoreClick = jest.fn();
  const root = render(
    <TreeButtonGroup
      handleResetClick={handleResetClick}
      handleClearClick={handleClearClick}
      handleRestoreClick={handleRestoreClick}
    />,
  );
  fireEvent.click(root.container.querySelector('button#clear-button'));
  fireEvent.click(root.container.querySelector('button#reset-button'));
  fireEvent.click(root.container.querySelector('button#restore-button'));

  expect(handleResetClick).toHaveBeenCalledTimes(1);
  expect(handleClearClick).toHaveBeenCalledTimes(1);
  expect(handleRestoreClick).toHaveBeenCalledTimes(1);
});
