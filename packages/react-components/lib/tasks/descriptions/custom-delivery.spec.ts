import {
  CustomDeliveryTaskDescription,
  makeCustomDeliveryTaskDescription,
} from './custom-delivery';

describe('Custom deliveries', () => {
  it('delivery 1:1', () => {
    let deliveryTaskDescription: CustomDeliveryTaskDescription | null = null;
    try {
      deliveryTaskDescription = JSON.parse(`{
        "category": "delivery_pickup",
        "phases": [
          {
            "activity": {
              "category": "sequence",
              "description": {
                "activities": [
                  {
                    "category": "go_to_place",
                    "description": "test_pickup_place"
                  },
                  {
                    "category": "perform_action",
                    "description": {
                      "unix_millis_action_duration_estimate": 60000,
                      "category": "delivery_pickup",
                      "description": {
                        "cart_id": "test_cart_id",
                        "pickup_lot": "test_pickup_lot"
                      }
                    }
                  }
                ]
              }
            }
          },
          {
            "activity": {
              "category": "sequence",
              "description": {
                "activities": [
                  {
                    "category": "go_to_place",
                    "description": "test_dropoff_place"
                  }
                ]
              }
            },
            "on_cancel": [
              {
                "category": "sequence",
                "description": [
                  {
                    "category": "go_to_place",
                    "description": {
                      "one_of": [
                        {
                          "waypoint": "test_waypoint_1"
                        },
                        {
                          "waypoint": "test_waypoint_2"
                        },
                        {
                          "waypoint": "test_waypoint_3"
                        }
                      ],
                      "constraints": [
                        {
                          "category": "prefer_same_map",
                          "description": ""
                        }
                      ]
                    }
                  },
                  {
                    "category": "perform_action",
                    "description": {
                      "unix_millis_action_duration_estimate": 60000,
                      "category": "delivery_dropoff",
                      "description": {}
                    }
                  }
                ]
              }
            ]
          },
          {
            "activity": {
              "category": "sequence",
              "description": {
                "activities": [
                  {
                    "category": "perform_action",
                    "description": {
                      "unix_millis_action_duration_estimate": 60000,
                      "category": "delivery_dropoff",
                      "description": {}
                    }
                  }
                ]
              }
            }
          }
        ]
      }
      `) as CustomDeliveryTaskDescription;
    } catch (e) {
      deliveryTaskDescription = null;
    }
    expect(deliveryTaskDescription).not.toEqual(null);
    expect(deliveryTaskDescription).toEqual(
      makeCustomDeliveryTaskDescription(
        'delivery_pickup',
        'test_pickup_place',
        'test_cart_id',
        'test_pickup_lot',
        'test_dropoff_place',
        ['test_waypoint_1', 'test_waypoint_2', 'test_waypoint_3'],
      ),
    );
  });

  it('delivery_sequential_lot_pickup', () => {
    let deliveryTaskDescription: CustomDeliveryTaskDescription | null = null;
    try {
      deliveryTaskDescription = JSON.parse(`{
        "category": "delivery_sequential_lot_pickup",
        "phases": [
          {
            "activity": {
              "category": "sequence",
              "description": {
                "activities": [
                  {
                    "category": "go_to_place",
                    "description": "test_pickup_place"
                  },
                  {
                    "category": "perform_action",
                    "description": {
                      "unix_millis_action_duration_estimate": 60000,
                      "category": "delivery_sequential_lot_pickup",
                      "description": {
                        "cart_id": "test_cart_id",
                        "pickup_zone": "test_pickup_zone"
                      }
                    }
                  }
                ]
              }
            }
          },
          {
            "activity": {
              "category": "sequence",
              "description": {
                "activities": [
                  {
                    "category": "go_to_place",
                    "description": "test_dropoff_place"
                  }
                ]
              }
            },
            "on_cancel": [
              {
                "category": "sequence",
                "description": [
                  {
                    "category": "go_to_place",
                    "description": {
                      "one_of": [
                        {
                          "waypoint": "test_waypoint_1"
                        },
                        {
                          "waypoint": "test_waypoint_2"
                        },
                        {
                          "waypoint": "test_waypoint_3"
                        }
                      ],
                      "constraints": [
                        {
                          "category": "prefer_same_map",
                          "description": ""
                        }
                      ]
                    }
                  },
                  {
                    "category": "perform_action",
                    "description": {
                      "unix_millis_action_duration_estimate": 60000,
                      "category": "delivery_dropoff",
                      "description": {}
                    }
                  }
                ]
              }
            ]
          },
          {
            "activity": {
              "category": "sequence",
              "description": {
                "activities": [
                  {
                    "category": "perform_action",
                    "description": {
                      "unix_millis_action_duration_estimate": 60000,
                      "category": "delivery_dropoff",
                      "description": {}
                    }
                  }
                ]
              }
            }
          }
        ]
      }
      `) as CustomDeliveryTaskDescription;
    } catch (e) {
      deliveryTaskDescription = null;
    }
    expect(deliveryTaskDescription).not.toEqual(null);
    expect(deliveryTaskDescription).toEqual(
      makeCustomDeliveryTaskDescription(
        'delivery_sequential_lot_pickup',
        'test_pickup_place',
        'test_cart_id',
        'test_pickup_zone',
        'test_dropoff_place',
        ['test_waypoint_1', 'test_waypoint_2', 'test_waypoint_3'],
      ),
    );
  });

  it('delivery_area_pickup', () => {
    let deliveryTaskDescription: CustomDeliveryTaskDescription | null = null;
    try {
      deliveryTaskDescription = JSON.parse(`{
        "category": "delivery_area_pickup",
        "phases": [
          {
            "activity": {
              "category": "sequence",
              "description": {
                "activities": [
                  {
                    "category": "go_to_place",
                    "description": "test_pickup_place"
                  },
                  {
                    "category": "perform_action",
                    "description": {
                      "unix_millis_action_duration_estimate": 60000,
                      "category": "delivery_area_pickup",
                      "description": {
                        "cart_id": "test_cart_id",
                        "pickup_zone": "test_pickup_zone"
                      }
                    }
                  }
                ]
              }
            }
          },
          {
            "activity": {
              "category": "sequence",
              "description": {
                "activities": [
                  {
                    "category": "go_to_place",
                    "description": "test_dropoff_place"
                  }
                ]
              }
            },
            "on_cancel": [
              {
                "category": "sequence",
                "description": [
                  {
                    "category": "go_to_place",
                    "description": {
                      "one_of": [
                        {
                          "waypoint": "test_waypoint_1"
                        },
                        {
                          "waypoint": "test_waypoint_2"
                        },
                        {
                          "waypoint": "test_waypoint_3"
                        }
                      ],
                      "constraints": [
                        {
                          "category": "prefer_same_map",
                          "description": ""
                        }
                      ]
                    }
                  },
                  {
                    "category": "perform_action",
                    "description": {
                      "unix_millis_action_duration_estimate": 60000,
                      "category": "delivery_dropoff",
                      "description": {}
                    }
                  }
                ]
              }
            ]
          },
          {
            "activity": {
              "category": "sequence",
              "description": {
                "activities": [
                  {
                    "category": "perform_action",
                    "description": {
                      "unix_millis_action_duration_estimate": 60000,
                      "category": "delivery_dropoff",
                      "description": {}
                    }
                  }
                ]
              }
            }
          }
        ]
      }
      `) as CustomDeliveryTaskDescription;
    } catch (e) {
      deliveryTaskDescription = null;
    }
    expect(deliveryTaskDescription).not.toEqual(null);
    expect(deliveryTaskDescription).toEqual(
      makeCustomDeliveryTaskDescription(
        'delivery_area_pickup',
        'test_pickup_place',
        'test_cart_id',
        'test_pickup_zone',
        'test_dropoff_place',
        ['test_waypoint_1', 'test_waypoint_2', 'test_waypoint_3'],
      ),
    );
  });
});
