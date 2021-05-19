import { render } from '@testing-library/react';
import React from 'react';
import { TaskPage } from '../task-page';
import { ThemeProvider } from '@material-ui/core';
import { lightTheme } from 'react-components';

describe('TaskPage', () => {
  it('smoke test', () => {
    render(
      <ThemeProvider theme={lightTheme}>
        <TaskPage />
      </ThemeProvider>,
    );
  });
});
