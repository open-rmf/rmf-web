import { makeStyles } from '@material-ui/core';
import AccordionDetails, { AccordionDetailsProps } from '@material-ui/core/AccordionDetails';
import React from 'react';
import { joinClasses } from './css-utils';

const useStyles = makeStyles((theme) => ({
  details: {
    flexFlow: 'column',
    padding: theme.spacing(1),
  },
}));

export type ItemAccordionDetailsProps = AccordionDetailsProps;

export const ItemAccordionDetails = (props: ItemAccordionDetailsProps): JSX.Element => {
  const { className, ...otherProps } = props;
  const classes = useStyles();
  return (
    <AccordionDetails
      className={joinClasses(classes.details, className)}
      {...otherProps}
    ></AccordionDetails>
  );
};

export default ItemAccordionDetails;
