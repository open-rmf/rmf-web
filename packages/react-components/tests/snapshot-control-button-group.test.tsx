import React from 'react';
import { SnapshotControlButtonGroup } from '../lib';
import { render } from '@testing-library/react';
import userEvent from '@testing-library/user-event';

test('It disabled buttons', () => {
  const root = render(
    <SnapshotControlButtonGroup disableClear={true} disableReset={true} disableRestore={true} />,
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
    <SnapshotControlButtonGroup
      handleResetClick={handleResetClick}
      handleClearClick={handleClearClick}
      handleRestoreClick={handleRestoreClick}
    />,
  );

  userEvent.click(root.getByText('Clear'));
  userEvent.click(root.getByText('Reset'));
  userEvent.click(root.getByText('Restore'));
  expect(handleResetClick).toHaveBeenCalledTimes(1);
  expect(handleClearClick).toHaveBeenCalledTimes(1);
  expect(handleRestoreClick).toHaveBeenCalledTimes(1);
});
