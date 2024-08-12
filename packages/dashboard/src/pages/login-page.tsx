import { styled } from '@mui/material';

import { LoginCard, LoginCardProps } from '../components';

const prefix = 'login-page';
const classes = {
  container: `${prefix}-container`,
};
const StyledDiv = styled('div')(({ theme }) => ({
  [`&.${classes.container}`]: {
    width: '100vw',
    height: '100vh',
    position: 'absolute',
    left: 0,
    top: 0,
    backgroundColor: theme.palette.primary.main,
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
  },
}));

export const LoginPage = (props: LoginCardProps): JSX.Element => {
  return (
    <StyledDiv className={classes.container}>
      <LoginCard {...props} />
    </StyledDiv>
  );
};

export default LoginPage;
