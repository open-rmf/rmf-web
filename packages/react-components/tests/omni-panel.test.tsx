import { render } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { OmniPanel, OmniPanelView } from '../lib';
import { OmniPanelProps } from '../lib/omni-panel';

function TestPanel(props: Omit<OmniPanelProps, 'stack' | 'children'>) {
  return (
    <OmniPanel stack={[0]} {...props}>
      <OmniPanelView viewId={0}></OmniPanelView>
    </OmniPanel>
  );
}

test('triggers onBack callback', () => {
  const handler = jest.fn();
  const root = render(<TestPanel onBack={handler} />);
  userEvent.click(root.getByLabelText('Back'));
  expect(handler).toHaveBeenCalled();
});

test('triggers onHome callback', () => {
  const handler = jest.fn();
  const root = render(<TestPanel onHome={handler} />);
  userEvent.click(root.getByLabelText('Home'));
  expect(handler).toHaveBeenCalled();
});

test('triggers onClose callback', () => {
  const handler = jest.fn();
  const root = render(<TestPanel variant="backHomeClose" onClose={handler} />);
  userEvent.click(root.getByLabelText('Close'));
  expect(handler).toHaveBeenCalled();
});

test('only render current view', () => {
  const root = render(
    <OmniPanel stack={[0]}>
      <OmniPanelView viewId={0}>
        <div data-testid="0"></div>
      </OmniPanelView>
      <OmniPanelView viewId={1}>
        <div data-testid="1"></div>
      </OmniPanelView>
    </OmniPanel>,
  );

  expect(window.getComputedStyle(root.getByTestId('0')).visibility).toBe('visible');
  // unmount when not in view
  expect(root.queryByTestId('1')).toBeFalsy();
});
