import React from 'react';
import { TextField, Typography, Divider } from '@material-ui/core';
import Autocomplete from '@material-ui/lab/Autocomplete';

import { styleTyping } from './Utils';

interface AutoCompleteProps {
    errorState: boolean[];
    errorMessage: string[];
    labels: string[];
}

const styles: styleTyping = {
    root: {
        margin: '0 auto',
        width: '40%',
    },
    heading: {
        padding: '0.5rem'
    },
    autoComplete: {
        padding: '1rem 0.5rem'
    }
}

export default function AutoCompleteComponent(props: AutoCompleteProps) {

    const { errorState, errorMessage, labels } = props;

    const fleetNames: string[] = [
        'fleet1',
        'fleet2',
        'fleet3'
    ];
    
    return (
        <div style={styles.root}>
            <div style={styles.heading}>
                <Typography variant="body1" >
                    Shown here is the combo box that we use for the selection of
                    a list of items. The examples below shows how the combo box appears 
                    when inputs are normal and errornous.
                </Typography>
            </div>
            {
                errorState.map((state, index) => {
                    return (
                        <React.Fragment key={index + `${state}`}>
                            <Divider />
                            <div style={styles.autoComplete}>
                                <Autocomplete
                                    getOptionLabel={option => option}
                                    options={fleetNames}
                                    renderInput={params => (
                                        <TextField
                                        {...params}
                                        label={labels[index]}
                                        variant="outlined"
                                        error={state}
                                        helperText={errorMessage[index]}
                                        name="targetFleet"
                                        />
                                    )}
                                />
                            </div>
                        </React.Fragment>
                    )
                })
            }
        </div>
    )
}
