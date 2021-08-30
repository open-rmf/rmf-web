import { makeStyles } from '@material-ui/core';
import AccordionDetails, { AccordionDetailsProps } from '@material-ui/core/AccordionDetails';
import clsx from 'clsx';
import React from 'react';

const useStyles = makeStyles({
  details: {
    flexFlow: 'column',
    padding: 0,
  },
});

export type ItemAccordionDetailsProps = AccordionDetailsProps;

export const ItemAccordionDetails = (props: ItemAccordionDetailsProps): JSX.Element => {
  const { className, ...otherProps } = props;
  const classes = useStyles();
  return (
    <AccordionDetails
      className={clsx(classes.details, className)}
      {...otherProps}
    ></AccordionDetails>
  );
};

export default ItemAccordionDetails;
