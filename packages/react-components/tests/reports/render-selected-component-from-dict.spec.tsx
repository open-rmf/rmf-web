import React from 'react';
import { cleanup, render, screen } from '@testing-library/react';
import { RenderSelectedComponentFromDict } from '../../lib';

describe('RenderSelectedComponentFromDict', () => {
  afterEach(() => cleanup());

  it('renders correctly', () => {
    const test = {
      test: <h1> test</h1>,
    };
    render(<RenderSelectedComponentFromDict selectedKey={'test'} obj={test} />);
    expect(screen.getByText('test')).toBeTruthy();
  });

  it('render nothing on null value', () => {
    const test = {
      test: null,
    };
    render(<RenderSelectedComponentFromDict selectedKey={'test'} obj={test} />);
    expect(screen.queryByText('test')).toBeFalsy();
  });

  it('render nothing on missing attribute', () => {
    const test = {};
    render(<RenderSelectedComponentFromDict selectedKey={'test'} obj={test} />);
    expect(screen.queryByText('test')).toBeFalsy();
  });
});
