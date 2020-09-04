import React from 'react';
import styled from 'styled-components';
import { Box, Button, Typography } from '@material-ui/core';

export const StyledDiv = styled(({ background, color, ...otherProps }) => <Box {...otherProps} />)`
  background: ${props => props.background};
  color: ${props => props.color};
`;

export const StyledButton = styled(({ color, ...otherProps }) => (
  <Button variant="contained" {...otherProps} />
))`
  color: ${props => props.color};
`;

export const HighlightedWords = styled(({ color, ...otherProps }) => (
  <Typography variant={'h6'} display={'inline'} {...otherProps} />
))`
  color: ${props => props.color};
  background-color: darkblue;
`;
