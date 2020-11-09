import React from 'react';
import { TreeButtonGroup } from '../lib';
import { render } from '@testing-library/react';

test('It disabled buttons', () => {
  const root = render(
    <TreeButtonGroup disableClear={true} disableReset={true} disableRestore={true} />,
  );
  expect(root.container.querySelector('button#clear-button')).toEqual(true);
  expect(root.container.querySelector('button#reset-button').props().disabled).toEqual(true);
  expect(root.container.querySelector('button#restore-button').props().disabled).toEqual(true);
});

// test('It executes callbacks correctly', () => {
//   const handleResetClick = jest.fn();
//   const handleClearClick = jest.fn();
//   const handleRestoreClick = jest.fn();
//   const root = mount(
//     <TreeButtonGroup
//       handleResetClick={handleRes\etClick}
//       handleClearClick={handleClearClick}
//       handleRestoreClick={handleRestoreClick}
//     />,
//   );
//   root.find('button#clear-button').simulate('click');
//   root.find('button#reset-button').simulate('click');
//   root.find('button#restore-button').simulate('click');

//   expect(handleResetClick).toHaveBeenCalledTimes(1);
//   expect(handleClearClick).toHaveBeenCalledTimes(1);
//   expect(handleRestoreClick).toHaveBeenCalledTimes(1);
// });
