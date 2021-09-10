import React from 'react';
import { makeStyles, Typography } from '@material-ui/core';
import type { Link } from 'react-router-dom';

export interface NotFoundPageProps {
  /**
   * The link to redirect user to the correct page.
   */
  linkComponent: React.ReactElement<Link>;
}

export const NotFoundPage = (props: NotFoundPageProps): React.ReactElement => {
  const { linkComponent: routeLinkComponent } = props;
  const classes = useStyles();
  console.warn('Photo by Aron Visuals on Unsplash');
  return (
    <div className={classes.div}>
      <img
        className={classes.img}
        src={'assets/aron-visuals-3jBU9TbKW7o-unsplash.jpg'}
        alt="404 Not Found"
      />
      <Typography variant="h1" component="h2" gutterBottom className={classes.text}>
        Are you lost? {routeLinkComponent}
      </Typography>
      <Typography className={classes.author}>Photo by Aron Visuals on Unsplash</Typography>
    </div>
  );
};

const useStyles = makeStyles(() => ({
  div: {
    width: '100%',
    height: '100%',
    overflowX: 'hidden',
    overflowY: 'hidden',
  },
  img: {
    width: '100%',
    height: '100%',
  },
  text: {
    position: 'fixed',
    left: '20%',
    top: '2%',
    color: 'white',
    fontWeight: 'bold',
  },
  author: {
    position: 'fixed',
    right: '5%',
    bottom: '2%',
    color: 'white',
  },
}));

export default NotFoundPage;
