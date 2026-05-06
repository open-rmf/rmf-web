import { Box, LinearProgress, LinearProgressProps, Typography } from '@mui/material';

export function LinearProgressBar(props: LinearProgressProps & { value: number }): JSX.Element {
  return (
    <Box display="flex" alignItems="center">
      <Box width="100%" mr={1}>
        <LinearProgress color="secondary" variant="determinate" {...props} />
      </Box>
      <Box minWidth={35}>
        <Typography variant="body2">{`${Math.floor(props.value)}%`}</Typography>
      </Box>
    </Box>
  );
}
