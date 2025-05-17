## Control flow of protocol

1. L joins WiFi
2. L setup steps (pinmodes, init lora, init display)
3. L configs NTP
4. C starts server
5. L connects to C
6. L sends magic#,CID
7. C waits until N devices connected
8. C sends start to all devices
9. L turns on radio and occasionally sends data
10. C sends stop after the timer ends
11. L sends rest of data
12. L sends null term measurement
13. C sends done
14. C and L close socket
15. L resets state in some fashion
16. C unifies the data into one file and exports


## Future work

* control TX
* have hard cut off for devices just in case they hang
* add indefinite mode (requires next task)
* add better ctrl+C behavior
* stagger device uploads
* tqdm in python for timer feedback?
* add robustness for wifi/socket disconnects
