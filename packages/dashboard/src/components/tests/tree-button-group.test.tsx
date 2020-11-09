import { mount, shallow } from 'enzyme';
import React from 'react';
import { TreeButtonGroup } from '../tree-button-group';

test('Matches snapshot', () => {
  const root = shallow(<TreeButtonGroup />);
  expect(root).toMatchSnapshot();
});

test('It disabled buttons', () => {
  const root = mount(
    <TreeButtonGroup disableClear={true} disableReset={true} disableRestore={true} />,
  );
  expect(root.find('button#clear-button').props().disabled).toEqual(true);
  expect(root.find('button#reset-button').props().disabled).toEqual(true);
  expect(root.find('button#restore-button').props().disabled).toEqual(true);
});

test('It executes callbacks correctly', () => {
  const handleResetClick = jest.fn();
  const handleClearClick = jest.fn();
  const handleRestoreClick = jest.fn();
  const root = mount(
    <TreeButtonGroup
      handleResetClick={handleResetClick}
      handleClearClick={handleClearClick}
      handleRestoreClick={handleRestoreClick}
    />,
  );
  root.find('button#clear-button').simulate('click');
  root.find('button#reset-button').simulate('click');
  root.find('button#restore-button').simulate('click');

  expect(handleResetClick).toHaveBeenCalledTimes(1);
  expect(handleClearClick).toHaveBeenCalledTimes(1);
  expect(handleRestoreClick).toHaveBeenCalledTimes(1);
});
