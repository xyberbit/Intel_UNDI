This branch is used to integrate open source UEFI UNDI drivers supporting Intel(R) server network adapters to EDK2.

Branch owner:
Pawel Orlowski < pawel.orlowski@intel.com >
Maciej Rabeda < maciej.rabeda@intel.com >

## Feature Introduction
Drivers being introduced in this branch allow utilizing and controlling Intel(R) server network adapters through UNDI protocol.

#### Related modules
The following module is being introduced in this branch:
```
IntelUndiPkg - new package which will contain sources for all Intel(R) server network adapter drivers.
```

## Support scope
UEFI UNDI Drivers being introduced in this branch will support only basic UNDI functionality for Intel(R) server network adapters.
Branch will be updated with support for new devices after those devices are released to public.

Architecture: IA32, IPF, X64

## Known issues
| Issue | Status |
|:-----:|:------:|
| Incorrect exit flow in InitChild process | Open |

## Status
### 2019-12-19
* Aligned source code to Intel network driver release 24.3.