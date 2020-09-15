import { makeLauncher } from '../rmf-launcher';
import { login, overwriteClick, removeTextFromAutocomplete } from './utils';

import { createMount } from '@material-ui/core/test-utils';
import React from 'react';
import { Redirect } from 'react-router';
import { BrowserRouter } from 'react-router-dom';

describe('Loop request for negotiations', () => {
  const launcher = makeLauncher();

  before(async () => await launcher.launch());
  after(async () => await launcher.kill());

  before(() => overwriteClick());
  before(() => browser.url('/'));

  before(login);


  it('renders negotiation trajectory', () => {
    browser.setTimeout({ 'script': 120000 })
    
    $('[data-component=MainMenu] [data-item=Commands]').click();

    const loopForm = $('[data-component=LoopForm]');
    loopForm.click();
    
    $('input[name=startLocation]').waitForClickable();
    $('input[name=startLocation]').setValue(removeTextFromAutocomplete(10));
    $('input[name=startLocation]').setValue('pantry');
    $('.MuiAutocomplete-popper').click();

    $('input[name=finishLocation]').waitForClickable();
    $('input[name=finishLocation]').setValue(removeTextFromAutocomplete(20));
    $('input[name=finishLocation]').setValue('cubicle_1');
    $('.MuiAutocomplete-popper').click();

    $('input[name=numLoops]').waitForClickable();
    $('input[name=numLoops]').setValue(1);
    loopForm.$('button=Request').click();

    $('input[name=startLocation]').waitForClickable();
    $('input[name=startLocation]').setValue(removeTextFromAutocomplete(10));
    $('input[name=startLocation]').setValue('pantry');
    $('.MuiAutocomplete-popper').click();

    $('input[name=finishLocation]').waitForClickable();
    $('input[name=finishLocation]').setValue(removeTextFromAutocomplete(20));
    $('input[name=finishLocation]').setValue('cubicle_2');
    $('.MuiAutocomplete-popper').click();

    $('input[name=numLoops]').waitForClickable();
    $('input[name=numLoops]').setValue(1);
    loopForm.$('button=Request').click();

    const backButton = $('[name="back-button"]');
    backButton.click();
    
    $('[data-component=MainMenu] [data-item=Negotiations]').click();

    browser.waitUntil(
      () => $('[data-component=TreeItem]').isDisplayed() === true,
      //() => browser.getElementText('*=Conflict') !== undefined,
      {
        timeout: 60000,
        timeoutMsg: 'expected TreeItem to be not null!'
      }
    );
    console.log('done');
    
    var treeItem = $('[data-component=TreeItem]');
    expect(treeItem).toBeVisible();
    treeItem.click();
    
    var subTreeItem = treeItem.$('[data-component=TreeItem]');
    expect(subTreeItem).toBeVisible();
    subTreeItem.click();
  });

  // Disabled till we find out a way to locate that floor icon
  // Also figure out a way for a non-rejected negotiation status click
  /*
  it('renders negotiation trajectory', () => {
    $('[data-component=viewOptions]').moveTo();
    $('[data-component=NegotiationTrajCheckbox]').click();
    expect($('[data-component=RobotTrajectory]')).toBeVisible();
  });
  */
});
