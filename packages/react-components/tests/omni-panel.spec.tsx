import { render } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { OmniPanel, OmniPanelView } from '../lib';
import { OmniPanelProps } from '../lib/omni-panel';

function TestPanel(props: Omit<OmniPanelProps, 'view' | 'children'>) {
  return (
    <OmniPanel view={0} {...props}>
      <OmniPanelView viewId={0}></OmniPanelView>
    </OmniPanel>
  );
}

describe('OmniPanel component', () => {
  let mockOnClick: ReturnType<typeof jasmine.createSpy>;

  beforeEach(() => {
    mockOnClick = jasmine.createSpy();
  });

  it('triggers onBack callback', () => {
    const root = render(<TestPanel onBack={mockOnClick} />);
    userEvent.click(root.getByLabelText('Back'));
    expect(mockOnClick).toHaveBeenCalled();
  });

  it('triggers onHome callback', () => {
    const root = render(<TestPanel onHome={mockOnClick} />);
    userEvent.click(root.getByLabelText('Home'));
    expect(mockOnClick).toHaveBeenCalled();
  });

  it('triggers onClose callback', () => {
    const root = render(<TestPanel variant="backHomeClose" onClose={mockOnClick} />);
    userEvent.click(root.getByLabelText('Close'));
    expect(mockOnClick).toHaveBeenCalled();
  });

  it('only render current view', () => {
    const root = render(
      <OmniPanel view={0}>
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
});
