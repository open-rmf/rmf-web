import React from 'react';
import { TreeButtonGroup } from '../lib';
import { render, fireEvent } from '@testing-library/react';

test('It disabled buttons', () => {
  const root = render(
    <TreeButtonGroup disableClear={true} disableReset={true} disableRestore={true} />,
  );
  expect(
    root.container.querySelector('button#clear-button')?.hasAttribute('disabled'),
  ).toBeTruthy();
  expect(
    root.container.querySelector('button#reset-button')?.hasAttribute('disabled'),
  ).toBeTruthy();
  expect(
    root.container.querySelector('button#restore-button')?.hasAttribute('disabled'),
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

  const clearButton = root.container.querySelector('button#clear-button');
  const resetButton = root.container.querySelector('button#reset-button');
  const restoreButton = root.container.querySelector('button#restore-button');

  // FireEvent should not receive null
  clearButton && fireEvent.click(clearButton);
  resetButton && fireEvent.click(resetButton);
  restoreButton && fireEvent.click(restoreButton);

  expect(handleResetClick).toHaveBeenCalledTimes(1);
  expect(handleClearClick).toHaveBeenCalledTimes(1);
  expect(handleRestoreClick).toHaveBeenCalledTimes(1);
});
