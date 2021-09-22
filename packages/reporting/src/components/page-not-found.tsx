import React from 'react';
import { styled, Typography } from '@material-ui/core';
import type { Link } from 'react-router-dom';

export interface NotFoundPageProps {
  /**
   * The link to redirect user to the correct page.
   */
  linkComponent: React.ReactElement<Link>;
}

const classes = {
  div: 'page-not-found-root',
  img: 'page-not-found-img',
  text: 'page-not-found-text',
  author: 'page-not-found-author',
};

const PageNotRoot = styled('div')(() => ({
  [`&.${classes.div}`]: {
    width: '100%',
    height: '100%',
    overflowX: 'hidden',
    overflowY: 'hidden',
  },
  [`& .${classes.img}`]: {
    width: '100%',
    height: '100%',
  },
  [`& .${classes.text}`]: {
    position: 'fixed',
    left: '20%',
    top: '2%',
    color: 'white',
    fontWeight: 'bold',
  },
  [`& .${classes.author}`]: {
    position: 'fixed',
    right: '5%',
    bottom: '2%',
    color: 'white',
  },
}));

export const NotFoundPage = (props: NotFoundPageProps): React.ReactElement => {
  const { linkComponent: routeLinkComponent } = props;
  console.warn('Photo by Aron Visuals on Unsplash');
  return (
    <PageNotRoot className={classes.div}>
      <img
        className={classes.img}
        src={'assets/aron-visuals-3jBU9TbKW7o-unsplash.jpg'}
        alt="404 Not Found"
      />
      <Typography variant="h1" component="h2" gutterBottom className={classes.text}>
        Are you lost? {routeLinkComponent}
      </Typography>
      <Typography className={classes.author}>Photo by Aron Visuals on Unsplash</Typography>
    </PageNotRoot>
  );
};

export default NotFoundPage;
