import React, { useState } from 'react';
import { useForm, Controller } from 'react-hook-form';
import Box from '@material-ui/core/Box';
import { TextField, makeStyles, Button } from '@material-ui/core';

interface defaultValuesInterface {
  firstName: string;
  lastName: string;
  email: string;
}

const defaultValues = {
  firstName: '',
  lastName: '',
  email: '',
};

const RegisterForm = function() {
  const { handleSubmit, register, reset, control } = useForm({ defaultValues });
  const [data, setData] = useState<defaultValuesInterface | null>(null);
  const classes = useStyle();
  return (
    <div className={classes.mainContainer}>
      <Box boxShadow={3} className={classes.boxContainer}>
        <div className={classes.logoContainer}>
          <img src="assets/ros-health.png" alt="" className={classes.logo} />
        </div>
        <form onSubmit={handleSubmit(data => setData(data))} className={classes.form}>
          <Controller
            as={TextField}
            name="email"
            type="email"
            label="Email Address"
            // disabled={isSubmitting}
            control={control}
          />
          <Controller as={TextField} name="firstName" label="First Name" control={control} />
          <Controller as={TextField} name="lastName" label="Last Name" control={control} />
          <Controller
            as={TextField}
            name="password"
            type="password"
            label="Password"
            control={control}
          />

          <div className={classes.buttonContainer}>
            <Button
              variant="contained"
              color="primary"
              type="submit"
              className={classes.buttonContainer}
            >
              Register
            </Button>
          </div>
        </form>
      </Box>
      <div className={classes.termOfServices}>
        By creating an account you agree to the RoMi Terms of Service.
      </div>
    </div>
  );
};

export default RegisterForm;

const useStyle = makeStyles(() => ({
  form: {
    display: 'flex',
    flexDirection: 'column',
    justifyContent: 'center',
    alignItems: 'center',
  },

  mainContainer: {
    minWidth: 275,
    display: 'flex',
    flexDirection: 'column',
    justifyContent: 'center',
    alignItems: 'center',
    height: '100%',
    fontFamily: 'monospace',
  },

  boxContainer: {
    minWidth: 275,
    display: 'flex',
    flexDirection: 'column',
    justifyContent: 'center',
    alignItems: 'center',
    padding: '0rem 2.5rem 0rem 2.5rem',
    borderRadius: 20,
    height: '40%',
  },

  buttonContainer: {
    marginTop: '0.5rem',
    width: '100%',
  },
  button: {
    width: '100%',
  },
  password: {},
  termOfServices: {
    alignSelf: 'center',
    color: '#918e8e',
    paddingTop: '0.5rem',
  },
  logo: {
    height: '100%',
  },
  logoContainer: {
    height: '10%',
    display: 'flex',
    justifyContent: 'center',
    alignItems: 'center',
    paddingBottom: '1rem',
  },
}));
