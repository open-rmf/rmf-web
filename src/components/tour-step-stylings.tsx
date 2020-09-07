import React from 'react';
import styled from 'styled-components';
import { Button, IconButton } from '@material-ui/core';
import {
  NavigateNext as NavigateNextIcon,
  NavigateBefore as NavigateBeforeIcon,
} from '@material-ui/icons';

export const LastStepNextButton = styled(({ ...otherProps }) => (
  <Button variant="contained" color="primary">
    Start using RoMi
  </Button>
))``;

export const NextButton = styled(({ ...otherProps }) => (
  <IconButton {...otherProps}>
    <NavigateNextIcon />
  </IconButton>
))``;

export const PrevButton = styled(({ ...otherProps }) => (
  <IconButton {...otherProps}>
    <NavigateBeforeIcon />
  </IconButton>
))``;
