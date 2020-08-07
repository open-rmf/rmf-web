import React from 'react';
import toJson from 'enzyme-to-json';
import { createMount } from '@material-ui/core/test-utils';

import OmniPanelStatusLabels, { LabelProps } from '../omni-panel-status-labels';

const mount = createMount();

const buildLabelsWrapper = (
  Component: (props: LabelProps) => React.ReactElement,
  modeText: string,
  modalLabelClass: string,
  name: string,
  hideTextStyleOverride?: string,
) => {
  return mount(
    <Component
      modeText={modeText}
      modalLabelClass={modalLabelClass}
      name={name}
      hideTextStyleOverride={hideTextStyleOverride}
    />,
  );
};

test('Component renders correctly', () => {
  expect(
    toJson(buildLabelsWrapper(OmniPanelStatusLabels, 'State', 'mock-style', 'Robot')),
  ).toMatchSnapshot();

  expect(
    toJson(
      buildLabelsWrapper(OmniPanelStatusLabels, 'State', 'mock-style', 'Robot', 'override-style'),
    ),
  ).toMatchSnapshot();
});

test('Component default style is being override when alternative style is provided', () => {
  const overrideStyle: string = 'override-style';
  const wrapper = buildLabelsWrapper(
    OmniPanelStatusLabels,
    'State',
    'mock-style',
    'Robot',
    overrideStyle,
  );

  const isOverride: boolean = wrapper
    .find(OmniPanelStatusLabels)
    .at(0)
    .find('h6')
    .hasClass(overrideStyle);

  expect(isOverride).toBe(true);
});
