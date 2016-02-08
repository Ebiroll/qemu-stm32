/*
 * This file is taken from gnutls 1.6.3 under the GPLv2+
 * and is under copyright of various GNUTLS contributors.
 */

#include "qemu/osdep.h"
#include <libtasn1.h>

const ASN1_ARRAY_TYPE pkix_asn1_tab[] = {
  {"PKIX1", 536875024, 0},
  {0, 1073741836, 0},
  {"id-ce", 1879048204, 0},
  {"joint-iso-ccitt", 1073741825, "2"},
  {"ds", 1073741825, "5"},
  {0, 1, "29"},
  {"id-ce-authorityKeyIdentifier", 1879048204, 0},
  {0, 1073741825, "id-ce"},
  {0, 1, "35"},
  {"AuthorityKeyIdentifier", 1610612741, 0},
  {"keyIdentifier", 1610637314, "KeyIdentifier"},
  {0, 4104, "0"},
  {"authorityCertIssuer", 1610637314, "GeneralNames"},
  {0, 4104, "1"},
  {"authorityCertSerialNumber", 536895490, "CertificateSerialNumber"},
  {0, 4104, "2"},
  {"KeyIdentifier", 1073741831, 0},
  {"id-ce-subjectKeyIdentifier", 1879048204, 0},
  {0, 1073741825, "id-ce"},
  {0, 1, "14"},
  {"SubjectKeyIdentifier", 1073741826, "KeyIdentifier"},
  {"id-ce-keyUsage", 1879048204, 0},
  {0, 1073741825, "id-ce"},
  {0, 1, "15"},
  {"KeyUsage", 1610874886, 0},
  {"digitalSignature", 1073741825, "0"},
  {"nonRepudiation", 1073741825, "1"},
  {"keyEncipherment", 1073741825, "2"},
  {"dataEncipherment", 1073741825, "3"},
  {"keyAgreement", 1073741825, "4"},
  {"keyCertSign", 1073741825, "5"},
  {"cRLSign", 1073741825, "6"},
  {"encipherOnly", 1073741825, "7"},
  {"decipherOnly", 1, "8"},
  {"id-ce-privateKeyUsagePeriod", 1879048204, 0},
  {0, 1073741825, "id-ce"},
  {0, 1, "16"},
  {"PrivateKeyUsagePeriod", 1610612741, 0},
  {"notBefore", 1619025937, 0},
  {0, 4104, "0"},
  {"notAfter", 545284113, 0},
  {0, 4104, "1"},
  {"id-ce-certificatePolicies", 1879048204, 0},
  {0, 1073741825, "id-ce"},
  {0, 1, "32"},
  {"CertificatePolicies", 1612709899, 0},
  {"MAX", 1074266122, "1"},
  {0, 2, "PolicyInformation"},
  {"PolicyInformation", 1610612741, 0},
  {"policyIdentifier", 1073741826, "CertPolicyId"},
  {"policyQualifiers", 538984459, 0},
  {"MAX", 1074266122, "1"},
  {0, 2, "PolicyQualifierInfo"},
  {"CertPolicyId", 1073741836, 0},
  {"PolicyQualifierInfo", 1610612741, 0},
  {"policyQualifierId", 1073741826, "PolicyQualifierId"},
  {"qualifier", 541065229, 0},
  {"policyQualifierId", 1, 0},
  {"PolicyQualifierId", 1073741836, 0},
  {"CPSuri", 1073741826, "IA5String"},
  {"UserNotice", 1610612741, 0},
  {"noticeRef", 1073758210, "NoticeReference"},
  {"explicitText", 16386, "DisplayText"},
  {"NoticeReference", 1610612741, 0},
  {"organization", 1073741826, "DisplayText"},
  {"noticeNumbers", 536870923, 0},
  {0, 3, 0},
  {"DisplayText", 1610612754, 0},
  {"visibleString", 1612709890, "VisibleString"},
  {"200", 524298, "1"},
  {"bmpString", 1612709890, "BMPString"},
  {"200", 524298, "1"},
  {"utf8String", 538968066, "UTF8String"},
  {"200", 524298, "1"},
  {"id-ce-policyMappings", 1879048204, 0},
  {0, 1073741825, "id-ce"},
  {0, 1, "33"},
  {"PolicyMappings", 1612709899, 0},
  {"MAX", 1074266122, "1"},
  {0, 536870917, 0},
  {"issuerDomainPolicy", 1073741826, "CertPolicyId"},
  {"subjectDomainPolicy", 2, "CertPolicyId"},
  {"DirectoryString", 1610612754, 0},
  {"teletexString", 1612709890, "TeletexString"},
  {"MAX", 524298, "1"},
  {"printableString", 1612709890, "PrintableString"},
  {"MAX", 524298, "1"},
  {"universalString", 1612709890, "UniversalString"},
  {"MAX", 524298, "1"},
  {"utf8String", 1612709890, "UTF8String"},
  {"MAX", 524298, "1"},
  {"bmpString", 1612709890, "BMPString"},
  {"MAX", 524298, "1"},
  {"ia5String", 538968066, "IA5String"},
  {"MAX", 524298, "1"},
  {"id-ce-subjectAltName", 1879048204, 0},
  {0, 1073741825, "id-ce"},
  {0, 1, "17"},
  {"SubjectAltName", 1073741826, "GeneralNames"},
  {"GeneralNames", 1612709899, 0},
  {"MAX", 1074266122, "1"},
  {0, 2, "GeneralName"},
  {"GeneralName", 1610612754, 0},
  {"otherName", 1610620930, "AnotherName"},
  {0, 4104, "0"},
  {"rfc822Name", 1610620930, "IA5String"},
  {0, 4104, "1"},
  {"dNSName", 1610620930, "IA5String"},
  {0, 4104, "2"},
  {"x400Address", 1610620930, "ORAddress"},
  {0, 4104, "3"},
  {"directoryName", 1610620930, "RDNSequence"},
  {0, 2056, "4"},
  {"ediPartyName", 1610620930, "EDIPartyName"},
  {0, 4104, "5"},
  {"uniformResourceIdentifier", 1610620930, "IA5String"},
  {0, 4104, "6"},
  {"iPAddress", 1610620935, 0},
  {0, 4104, "7"},
  {"registeredID", 536879116, 0},
  {0, 4104, "8"},
  {"AnotherName", 1610612741, 0},
  {"type-id", 1073741836, 0},
  {"value", 541073421, 0},
  {0, 1073743880, "0"},
  {"type-id", 1, 0},
  {"EDIPartyName", 1610612741, 0},
  {"nameAssigner", 1610637314, "DirectoryString"},
  {0, 4104, "0"},
  {"partyName", 536879106, "DirectoryString"},
  {0, 4104, "1"},
  {"id-ce-issuerAltName", 1879048204, 0},
  {0, 1073741825, "id-ce"},
  {0, 1, "18"},
  {"IssuerAltName", 1073741826, "GeneralNames"},
  {"id-ce-subjectDirectoryAttributes", 1879048204, 0},
  {0, 1073741825, "id-ce"},
  {0, 1, "9"},
  {"SubjectDirectoryAttributes", 1612709899, 0},
  {"MAX", 1074266122, "1"},
  {0, 2, "Attribute"},
  {"id-ce-basicConstraints", 1879048204, 0},
  {0, 1073741825, "id-ce"},
  {0, 1, "19"},
  {"BasicConstraints", 1610612741, 0},
  {"cA", 1610645508, 0},
  {0, 131081, 0},
  {"pathLenConstraint", 537411587, 0},
  {"0", 10, "MAX"},
  {"id-ce-nameConstraints", 1879048204, 0},
  {0, 1073741825, "id-ce"},
  {0, 1, "30"},
  {"NameConstraints", 1610612741, 0},
  {"permittedSubtrees", 1610637314, "GeneralSubtrees"},
  {0, 4104, "0"},
  {"excludedSubtrees", 536895490, "GeneralSubtrees"},
  {0, 4104, "1"},
  {"GeneralSubtrees", 1612709899, 0},
  {"MAX", 1074266122, "1"},
  {0, 2, "GeneralSubtree"},
  {"GeneralSubtree", 1610612741, 0},
  {"base", 1073741826, "GeneralName"},
  {"minimum", 1610653698, "BaseDistance"},
  {0, 1073741833, "0"},
  {0, 4104, "0"},
  {"maximum", 536895490, "BaseDistance"},
  {0, 4104, "1"},
  {"BaseDistance", 1611137027, 0},
  {"0", 10, "MAX"},
  {"id-ce-policyConstraints", 1879048204, 0},
  {0, 1073741825, "id-ce"},
  {0, 1, "36"},
  {"PolicyConstraints", 1610612741, 0},
  {"requireExplicitPolicy", 1610637314, "SkipCerts"},
  {0, 4104, "0"},
  {"inhibitPolicyMapping", 536895490, "SkipCerts"},
  {0, 4104, "1"},
  {"SkipCerts", 1611137027, 0},
  {"0", 10, "MAX"},
  {"id-ce-cRLDistributionPoints", 1879048204, 0},
  {0, 1073741825, "id-ce"},
  {0, 1, "31"},
  {"CRLDistributionPoints", 1612709899, 0},
  {"MAX", 1074266122, "1"},
  {0, 2, "DistributionPoint"},
  {"DistributionPoint", 1610612741, 0},
  {"distributionPoint", 1610637314, "DistributionPointName"},
  {0, 2056, "0"},
  {"reasons", 1610637314, "ReasonFlags"},
  {0, 4104, "1"},
  {"cRLIssuer", 536895490, "GeneralNames"},
  {0, 4104, "2"},
  {"DistributionPointName", 1610612754, 0},
  {"fullName", 1610620930, "GeneralNames"},
  {0, 4104, "0"},
  {"nameRelativeToCRLIssuer", 536879106, "RelativeDistinguishedName"},
  {0, 4104, "1"},
  {"ReasonFlags", 1610874886, 0},
  {"unused", 1073741825, "0"},
  {"keyCompromise", 1073741825, "1"},
  {"cACompromise", 1073741825, "2"},
  {"affiliationChanged", 1073741825, "3"},
  {"superseded", 1073741825, "4"},
  {"cessationOfOperation", 1073741825, "5"},
  {"certificateHold", 1073741825, "6"},
  {"privilegeWithdrawn", 1073741825, "7"},
  {"aACompromise", 1, "8"},
  {"id-ce-extKeyUsage", 1879048204, 0},
  {0, 1073741825, "id-ce"},
  {0, 1, "37"},
  {"ExtKeyUsageSyntax", 1612709899, 0},
  {"MAX", 1074266122, "1"},
  {0, 2, "KeyPurposeId"},
  {"KeyPurposeId", 1073741836, 0},
  {"id-kp-serverAuth", 1879048204, 0},
  {0, 1073741825, "id-kp"},
  {0, 1, "1"},
  {"id-kp-clientAuth", 1879048204, 0},
  {0, 1073741825, "id-kp"},
  {0, 1, "2"},
  {"id-kp-codeSigning", 1879048204, 0},
  {0, 1073741825, "id-kp"},
  {0, 1, "3"},
  {"id-kp-emailProtection", 1879048204, 0},
  {0, 1073741825, "id-kp"},
  {0, 1, "4"},
  {"id-kp-ipsecEndSystem", 1879048204, 0},
  {0, 1073741825, "id-kp"},
  {0, 1, "5"},
  {"id-kp-ipsecTunnel", 1879048204, 0},
  {0, 1073741825, "id-kp"},
  {0, 1, "6"},
  {"id-kp-ipsecUser", 1879048204, 0},
  {0, 1073741825, "id-kp"},
  {0, 1, "7"},
  {"id-kp-timeStamping", 1879048204, 0},
  {0, 1073741825, "id-kp"},
  {0, 1, "8"},
  {"id-pe-authorityInfoAccess", 1879048204, 0},
  {0, 1073741825, "id-pe"},
  {0, 1, "1"},
  {"AuthorityInfoAccessSyntax", 1612709899, 0},
  {"MAX", 1074266122, "1"},
  {0, 2, "AccessDescription"},
  {"AccessDescription", 1610612741, 0},
  {"accessMethod", 1073741836, 0},
  {"accessLocation", 2, "GeneralName"},
  {"id-ce-cRLNumber", 1879048204, 0},
  {0, 1073741825, "id-ce"},
  {0, 1, "20"},
  {"CRLNumber", 1611137027, 0},
  {"0", 10, "MAX"},
  {"id-ce-issuingDistributionPoint", 1879048204, 0},
  {0, 1073741825, "id-ce"},
  {0, 1, "28"},
  {"IssuingDistributionPoint", 1610612741, 0},
  {"distributionPoint", 1610637314, "DistributionPointName"},
  {0, 4104, "0"},
  {"onlyContainsUserCerts", 1610653700, 0},
  {0, 1073872905, 0},
  {0, 4104, "1"},
  {"onlyContainsCACerts", 1610653700, 0},
  {0, 1073872905, 0},
  {0, 4104, "2"},
  {"onlySomeReasons", 1610637314, "ReasonFlags"},
  {0, 4104, "3"},
  {"indirectCRL", 536911876, 0},
  {0, 1073872905, 0},
  {0, 4104, "4"},
  {"id-ce-deltaCRLIndicator", 1879048204, 0},
  {0, 1073741825, "id-ce"},
  {0, 1, "27"},
  {"BaseCRLNumber", 1073741826, "CRLNumber"},
  {"id-ce-cRLReasons", 1879048204, 0},
  {0, 1073741825, "id-ce"},
  {0, 1, "21"},
  {"CRLReason", 1610874901, 0},
  {"unspecified", 1073741825, "0"},
  {"keyCompromise", 1073741825, "1"},
  {"cACompromise", 1073741825, "2"},
  {"affiliationChanged", 1073741825, "3"},
  {"superseded", 1073741825, "4"},
  {"cessationOfOperation", 1073741825, "5"},
  {"certificateHold", 1073741825, "6"},
  {"removeFromCRL", 1, "8"},
  {"id-ce-certificateIssuer", 1879048204, 0},
  {0, 1073741825, "id-ce"},
  {0, 1, "29"},
  {"CertificateIssuer", 1073741826, "GeneralNames"},
  {"id-ce-holdInstructionCode", 1879048204, 0},
  {0, 1073741825, "id-ce"},
  {0, 1, "23"},
  {"HoldInstructionCode", 1073741836, 0},
  {"holdInstruction", 1879048204, 0},
  {"joint-iso-itu-t", 1073741825, "2"},
  {"member-body", 1073741825, "2"},
  {"us", 1073741825, "840"},
  {"x9cm", 1073741825, "10040"},
  {0, 1, "2"},
  {"id-holdinstruction-none", 1879048204, 0},
  {0, 1073741825, "holdInstruction"},
  {0, 1, "1"},
  {"id-holdinstruction-callissuer", 1879048204, 0},
  {0, 1073741825, "holdInstruction"},
  {0, 1, "2"},
  {"id-holdinstruction-reject", 1879048204, 0},
  {0, 1073741825, "holdInstruction"},
  {0, 1, "3"},
  {"id-ce-invalidityDate", 1879048204, 0},
  {0, 1073741825, "id-ce"},
  {0, 1, "24"},
  {"InvalidityDate", 1082130449, 0},
  {"VisibleString", 1610620935, 0},
  {0, 4360, "26"},
  {"NumericString", 1610620935, 0},
  {0, 4360, "18"},
  {"IA5String", 1610620935, 0},
  {0, 4360, "22"},
  {"TeletexString", 1610620935, 0},
  {0, 4360, "20"},
  {"PrintableString", 1610620935, 0},
  {0, 4360, "19"},
  {"UniversalString", 1610620935, 0},
  {0, 4360, "28"},
  {"BMPString", 1610620935, 0},
  {0, 4360, "30"},
  {"UTF8String", 1610620935, 0},
  {0, 4360, "12"},
  {"id-pkix", 1879048204, 0},
  {"iso", 1073741825, "1"},
  {"identified-organization", 1073741825, "3"},
  {"dod", 1073741825, "6"},
  {"internet", 1073741825, "1"},
  {"security", 1073741825, "5"},
  {"mechanisms", 1073741825, "5"},
  {"pkix", 1, "7"},
  {"id-pe", 1879048204, 0},
  {0, 1073741825, "id-pkix"},
  {0, 1, "1"},
  {"id-qt", 1879048204, 0},
  {0, 1073741825, "id-pkix"},
  {0, 1, "2"},
  {"id-kp", 1879048204, 0},
  {0, 1073741825, "id-pkix"},
  {0, 1, "3"},
  {"id-ad", 1879048204, 0},
  {0, 1073741825, "id-pkix"},
  {0, 1, "48"},
  {"id-qt-cps", 1879048204, 0},
  {0, 1073741825, "id-qt"},
  {0, 1, "1"},
  {"id-qt-unotice", 1879048204, 0},
  {0, 1073741825, "id-qt"},
  {0, 1, "2"},
  {"id-ad-ocsp", 1879048204, 0},
  {0, 1073741825, "id-ad"},
  {0, 1, "1"},
  {"id-ad-caIssuers", 1879048204, 0},
  {0, 1073741825, "id-ad"},
  {0, 1, "2"},
  {"Attribute", 1610612741, 0},
  {"type", 1073741826, "AttributeType"},
  {"values", 536870927, 0},
  {0, 2, "AttributeValue"},
  {"AttributeType", 1073741836, 0},
  {"AttributeValue", 1614807053, 0},
  {"type", 1, 0},
  {"AttributeTypeAndValue", 1610612741, 0},
  {"type", 1073741826, "AttributeType"},
  {"value", 2, "AttributeValue"},
  {"id-at", 1879048204, 0},
  {"joint-iso-ccitt", 1073741825, "2"},
  {"ds", 1073741825, "5"},
  {0, 1, "4"},
  {"id-at-initials", 1880096780, "AttributeType"},
  {0, 1073741825, "id-at"},
  {0, 1, "43"},
  {"X520initials", 1073741826, "DirectoryString"},
  {"id-at-generationQualifier", 1880096780, "AttributeType"},
  {0, 1073741825, "id-at"},
  {0, 1, "44"},
  {"X520generationQualifier", 1073741826, "DirectoryString"},
  {"id-at-surname", 1880096780, "AttributeType"},
  {0, 1073741825, "id-at"},
  {0, 1, "4"},
  {"X520surName", 1073741826, "DirectoryString"},
  {"id-at-givenName", 1880096780, "AttributeType"},
  {0, 1073741825, "id-at"},
  {0, 1, "42"},
  {"X520givenName", 1073741826, "DirectoryString"},
  {"id-at-name", 1880096780, "AttributeType"},
  {0, 1073741825, "id-at"},
  {0, 1, "41"},
  {"X520name", 1073741826, "DirectoryString"},
  {"id-at-commonName", 1880096780, "AttributeType"},
  {0, 1073741825, "id-at"},
  {0, 1, "3"},
  {"X520CommonName", 1073741826, "DirectoryString"},
  {"id-at-localityName", 1880096780, "AttributeType"},
  {0, 1073741825, "id-at"},
  {0, 1, "7"},
  {"X520LocalityName", 1073741826, "DirectoryString"},
  {"id-at-stateOrProvinceName", 1880096780, "AttributeType"},
  {0, 1073741825, "id-at"},
  {0, 1, "8"},
  {"X520StateOrProvinceName", 1073741826, "DirectoryString"},
  {"id-at-organizationName", 1880096780, "AttributeType"},
  {0, 1073741825, "id-at"},
  {0, 1, "10"},
  {"X520OrganizationName", 1073741826, "DirectoryString"},
  {"id-at-organizationalUnitName", 1880096780, "AttributeType"},
  {0, 1073741825, "id-at"},
  {0, 1, "11"},
  {"X520OrganizationalUnitName", 1073741826, "DirectoryString"},
  {"id-at-title", 1880096780, "AttributeType"},
  {0, 1073741825, "id-at"},
  {0, 1, "12"},
  {"X520Title", 1073741826, "DirectoryString"},
  {"id-at-description", 1880096780, "AttributeType"},
  {0, 1073741825, "id-at"},
  {0, 1, "13"},
  {"X520Description", 1073741826, "DirectoryString"},
  {"id-at-dnQualifier", 1880096780, "AttributeType"},
  {0, 1073741825, "id-at"},
  {0, 1, "46"},
  {"X520dnQualifier", 1073741826, "PrintableString"},
  {"id-at-countryName", 1880096780, "AttributeType"},
  {0, 1073741825, "id-at"},
  {0, 1, "6"},
  {"X520countryName", 1612709890, "PrintableString"},
  {0, 1048586, "2"},
  {"id-at-serialNumber", 1880096780, "AttributeType"},
  {0, 1073741825, "id-at"},
  {0, 1, "5"},
  {"X520serialNumber", 1073741826, "PrintableString"},
  {"id-at-telephoneNumber", 1880096780, "AttributeType"},
  {0, 1073741825, "id-at"},
  {0, 1, "20"},
  {"X520telephoneNumber", 1073741826, "PrintableString"},
  {"id-at-facsimileTelephoneNumber", 1880096780, "AttributeType"},
  {0, 1073741825, "id-at"},
  {0, 1, "23"},
  {"X520facsimileTelephoneNumber", 1073741826, "PrintableString"},
  {"id-at-pseudonym", 1880096780, "AttributeType"},
  {0, 1073741825, "id-at"},
  {0, 1, "65"},
  {"X520pseudonym", 1073741826, "DirectoryString"},
  {"id-at-name", 1880096780, "AttributeType"},
  {0, 1073741825, "id-at"},
  {0, 1, "41"},
  {"X520name", 1073741826, "DirectoryString"},
  {"id-at-streetAddress", 1880096780, "AttributeType"},
  {0, 1073741825, "id-at"},
  {0, 1, "9"},
  {"X520streetAddress", 1073741826, "DirectoryString"},
  {"id-at-postalAddress", 1880096780, "AttributeType"},
  {0, 1073741825, "id-at"},
  {0, 1, "16"},
  {"X520postalAddress", 1073741826, "PostalAddress"},
  {"PostalAddress", 1610612747, 0},
  {0, 2, "DirectoryString"},
  {"pkcs", 1879048204, 0},
  {"iso", 1073741825, "1"},
  {"member-body", 1073741825, "2"},
  {"us", 1073741825, "840"},
  {"rsadsi", 1073741825, "113549"},
  {"pkcs", 1, "1"},
  {"pkcs-9", 1879048204, 0},
  {0, 1073741825, "pkcs"},
  {0, 1, "9"},
  {"emailAddress", 1880096780, "AttributeType"},
  {0, 1073741825, "pkcs-9"},
  {0, 1, "1"},
  {"Pkcs9email", 1612709890, "IA5String"},
  {"ub-emailaddress-length", 524298, "1"},
  {"Name", 1610612754, 0},
  {"rdnSequence", 2, "RDNSequence"},
  {"RDNSequence", 1610612747, 0},
  {0, 2, "RelativeDistinguishedName"},
  {"DistinguishedName", 1073741826, "RDNSequence"},
  {"RelativeDistinguishedName", 1612709903, 0},
  {"MAX", 1074266122, "1"},
  {0, 2, "AttributeTypeAndValue"},
  {"Certificate", 1610612741, 0},
  {"tbsCertificate", 1073741826, "TBSCertificate"},
  {"signatureAlgorithm", 1073741826, "AlgorithmIdentifier"},
  {"signature", 6, 0},
  {"TBSCertificate", 1610612741, 0},
  {"version", 1610653698, "Version"},
  {0, 1073741833, "v1"},
  {0, 2056, "0"},
  {"serialNumber", 1073741826, "CertificateSerialNumber"},
  {"signature", 1073741826, "AlgorithmIdentifier"},
  {"issuer", 1073741826, "Name"},
  {"validity", 1073741826, "Validity"},
  {"subject", 1073741826, "Name"},
  {"subjectPublicKeyInfo", 1073741826, "SubjectPublicKeyInfo"},
  {"issuerUniqueID", 1610637314, "UniqueIdentifier"},
  {0, 4104, "1"},
  {"subjectUniqueID", 1610637314, "UniqueIdentifier"},
  {0, 4104, "2"},
  {"extensions", 536895490, "Extensions"},
  {0, 2056, "3"},
  {"Version", 1610874883, 0},
  {"v1", 1073741825, "0"},
  {"v2", 1073741825, "1"},
  {"v3", 1, "2"},
  {"CertificateSerialNumber", 1073741827, 0},
  {"Validity", 1610612741, 0},
  {"notBefore", 1073741826, "Time"},
  {"notAfter", 2, "Time"},
  {"Time", 1610612754, 0},
  {"utcTime", 1090519057, 0},
  {"generalTime", 8388625, 0},
  {"UniqueIdentifier", 1073741830, 0},
  {"SubjectPublicKeyInfo", 1610612741, 0},
  {"algorithm", 1073741826, "AlgorithmIdentifier"},
  {"subjectPublicKey", 6, 0},
  {"Extensions", 1612709899, 0},
  {"MAX", 1074266122, "1"},
  {0, 2, "Extension"},
  {"Extension", 1610612741, 0},
  {"extnID", 1073741836, 0},
  {"critical", 1610645508, 0},
  {0, 131081, 0},
  {"extnValue", 7, 0},
  {"CertificateList", 1610612741, 0},
  {"tbsCertList", 1073741826, "TBSCertList"},
  {"signatureAlgorithm", 1073741826, "AlgorithmIdentifier"},
  {"signature", 6, 0},
  {"TBSCertList", 1610612741, 0},
  {"version", 1073758210, "Version"},
  {"signature", 1073741826, "AlgorithmIdentifier"},
  {"issuer", 1073741826, "Name"},
  {"thisUpdate", 1073741826, "Time"},
  {"nextUpdate", 1073758210, "Time"},
  {"revokedCertificates", 1610629131, 0},
  {0, 536870917, 0},
  {"userCertificate", 1073741826, "CertificateSerialNumber"},
  {"revocationDate", 1073741826, "Time"},
  {"crlEntryExtensions", 16386, "Extensions"},
  {"crlExtensions", 536895490, "Extensions"},
  {0, 2056, "0"},
  {"AlgorithmIdentifier", 1610612741, 0},
  {"algorithm", 1073741836, 0},
  {"parameters", 541081613, 0},
  {"algorithm", 1, 0},
  {"pkcs-1", 1879048204, 0},
  {0, 1073741825, "pkcs"},
  {0, 1, "1"},
  {"rsaEncryption", 1879048204, 0},
  {0, 1073741825, "pkcs-1"},
  {0, 1, "1"},
  {"md2WithRSAEncryption", 1879048204, 0},
  {0, 1073741825, "pkcs-1"},
  {0, 1, "2"},
  {"md5WithRSAEncryption", 1879048204, 0},
  {0, 1073741825, "pkcs-1"},
  {0, 1, "4"},
  {"sha1WithRSAEncryption", 1879048204, 0},
  {0, 1073741825, "pkcs-1"},
  {0, 1, "5"},
  {"id-dsa-with-sha1", 1879048204, 0},
  {"iso", 1073741825, "1"},
  {"member-body", 1073741825, "2"},
  {"us", 1073741825, "840"},
  {"x9-57", 1073741825, "10040"},
  {"x9algorithm", 1073741825, "4"},
  {0, 1, "3"},
  {"Dss-Sig-Value", 1610612741, 0},
  {"r", 1073741827, 0},
  {"s", 3, 0},
  {"dhpublicnumber", 1879048204, 0},
  {"iso", 1073741825, "1"},
  {"member-body", 1073741825, "2"},
  {"us", 1073741825, "840"},
  {"ansi-x942", 1073741825, "10046"},
  {"number-type", 1073741825, "2"},
  {0, 1, "1"},
  {"DomainParameters", 1610612741, 0},
  {"p", 1073741827, 0},
  {"g", 1073741827, 0},
  {"q", 1073741827, 0},
  {"j", 1073758211, 0},
  {"validationParms", 16386, "ValidationParms"},
  {"ValidationParms", 1610612741, 0},
  {"seed", 1073741830, 0},
  {"pgenCounter", 3, 0},
  {"id-dsa", 1879048204, 0},
  {"iso", 1073741825, "1"},
  {"member-body", 1073741825, "2"},
  {"us", 1073741825, "840"},
  {"x9-57", 1073741825, "10040"},
  {"x9algorithm", 1073741825, "4"},
  {0, 1, "1"},
  {"Dss-Parms", 1610612741, 0},
  {"p", 1073741827, 0},
  {"q", 1073741827, 0},
  {"g", 3, 0},
  {"ORAddress", 1610612741, 0},
  {"built-in-standard-attributes", 1073741826, "BuiltInStandardAttributes"},
  {"built-in-domain-defined-attributes", 1073758210,
   "BuiltInDomainDefinedAttributes"},
  {"extension-attributes", 16386, "ExtensionAttributes"},
  {"BuiltInStandardAttributes", 1610612741, 0},
  {"country-name", 1073758210, "CountryName"},
  {"administration-domain-name", 1073758210, "AdministrationDomainName"},
  {"network-address", 1610637314, "NetworkAddress"},
  {0, 2056, "0"},
  {"terminal-identifier", 1610637314, "TerminalIdentifier"},
  {0, 2056, "1"},
  {"private-domain-name", 1610637314, "PrivateDomainName"},
  {0, 2056, "2"},
  {"organization-name", 1610637314, "OrganizationName"},
  {0, 2056, "3"},
  {"numeric-user-identifier", 1610637314, "NumericUserIdentifier"},
  {0, 2056, "4"},
  {"personal-name", 1610637314, "PersonalName"},
  {0, 2056, "5"},
  {"organizational-unit-names", 536895490, "OrganizationalUnitNames"},
  {0, 2056, "6"},
  {"CountryName", 1610620946, 0},
  {0, 1073746952, "1"},
  {"x121-dcc-code", 1612709890, "NumericString"},
  {0, 1048586, "ub-country-name-numeric-length"},
  {"iso-3166-alpha2-code", 538968066, "PrintableString"},
  {0, 1048586, "ub-country-name-alpha-length"},
  {"AdministrationDomainName", 1610620946, 0},
  {0, 1073744904, "2"},
  {"numeric", 1612709890, "NumericString"},
  {"ub-domain-name-length", 524298, "0"},
  {"printable", 538968066, "PrintableString"},
  {"ub-domain-name-length", 524298, "0"},
  {"NetworkAddress", 1073741826, "X121Address"},
  {"X121Address", 1612709890, "NumericString"},
  {"ub-x121-address-length", 524298, "1"},
  {"TerminalIdentifier", 1612709890, "PrintableString"},
  {"ub-terminal-id-length", 524298, "1"},
  {"PrivateDomainName", 1610612754, 0},
  {"numeric", 1612709890, "NumericString"},
  {"ub-domain-name-length", 524298, "1"},
  {"printable", 538968066, "PrintableString"},
  {"ub-domain-name-length", 524298, "1"},
  {"OrganizationName", 1612709890, "PrintableString"},
  {"ub-organization-name-length", 524298, "1"},
  {"NumericUserIdentifier", 1612709890, "NumericString"},
  {"ub-numeric-user-id-length", 524298, "1"},
  {"PersonalName", 1610612750, 0},
  {"surname", 1814044674, "PrintableString"},
  {0, 1073745928, "0"},
  {"ub-surname-length", 524298, "1"},
  {"given-name", 1814061058, "PrintableString"},
  {0, 1073745928, "1"},
  {"ub-given-name-length", 524298, "1"},
  {"initials", 1814061058, "PrintableString"},
  {0, 1073745928, "2"},
  {"ub-initials-length", 524298, "1"},
  {"generation-qualifier", 740319234, "PrintableString"},
  {0, 1073745928, "3"},
  {"ub-generation-qualifier-length", 524298, "1"},
  {"OrganizationalUnitNames", 1612709899, 0},
  {"ub-organizational-units", 1074266122, "1"},
  {0, 2, "OrganizationalUnitName"},
  {"OrganizationalUnitName", 1612709890, "PrintableString"},
  {"ub-organizational-unit-name-length", 524298, "1"},
  {"BuiltInDomainDefinedAttributes", 1612709899, 0},
  {"ub-domain-defined-attributes", 1074266122, "1"},
  {0, 2, "BuiltInDomainDefinedAttribute"},
  {"BuiltInDomainDefinedAttribute", 1610612741, 0},
  {"type", 1612709890, "PrintableString"},
  {"ub-domain-defined-attribute-type-length", 524298, "1"},
  {"value", 538968066, "PrintableString"},
  {"ub-domain-defined-attribute-value-length", 524298, "1"},
  {"ExtensionAttributes", 1612709903, 0},
  {"ub-extension-attributes", 1074266122, "1"},
  {0, 2, "ExtensionAttribute"},
  {"ExtensionAttribute", 1610612741, 0},
  {"extension-attribute-type", 1611145219, 0},
  {0, 1073743880, "0"},
  {"0", 10, "ub-extension-attributes"},
  {"extension-attribute-value", 541073421, 0},
  {0, 1073743880, "1"},
  {"extension-attribute-type", 1, 0},
  {"common-name", 1342177283, "1"},
  {"CommonName", 1612709890, "PrintableString"},
  {"ub-common-name-length", 524298, "1"},
  {"teletex-common-name", 1342177283, "2"},
  {"TeletexCommonName", 1612709890, "TeletexString"},
  {"ub-common-name-length", 524298, "1"},
  {"teletex-organization-name", 1342177283, "3"},
  {"TeletexOrganizationName", 1612709890, "TeletexString"},
  {"ub-organization-name-length", 524298, "1"},
  {"teletex-personal-name", 1342177283, "4"},
  {"TeletexPersonalName", 1610612750, 0},
  {"surname", 1814044674, "TeletexString"},
  {0, 1073743880, "0"},
  {"ub-surname-length", 524298, "1"},
  {"given-name", 1814061058, "TeletexString"},
  {0, 1073743880, "1"},
  {"ub-given-name-length", 524298, "1"},
  {"initials", 1814061058, "TeletexString"},
  {0, 1073743880, "2"},
  {"ub-initials-length", 524298, "1"},
  {"generation-qualifier", 740319234, "TeletexString"},
  {0, 1073743880, "3"},
  {"ub-generation-qualifier-length", 524298, "1"},
  {"teletex-organizational-unit-names", 1342177283, "5"},
  {"TeletexOrganizationalUnitNames", 1612709899, 0},
  {"ub-organizational-units", 1074266122, "1"},
  {0, 2, "TeletexOrganizationalUnitName"},
  {"TeletexOrganizationalUnitName", 1612709890, "TeletexString"},
  {"ub-organizational-unit-name-length", 524298, "1"},
  {"pds-name", 1342177283, "7"},
  {"PDSName", 1612709890, "PrintableString"},
  {"ub-pds-name-length", 524298, "1"},
  {"physical-delivery-country-name", 1342177283, "8"},
  {"PhysicalDeliveryCountryName", 1610612754, 0},
  {"x121-dcc-code", 1612709890, "NumericString"},
  {0, 1048586, "ub-country-name-numeric-length"},
  {"iso-3166-alpha2-code", 538968066, "PrintableString"},
  {0, 1048586, "ub-country-name-alpha-length"},
  {"postal-code", 1342177283, "9"},
  {"PostalCode", 1610612754, 0},
  {"numeric-code", 1612709890, "NumericString"},
  {"ub-postal-code-length", 524298, "1"},
  {"printable-code", 538968066, "PrintableString"},
  {"ub-postal-code-length", 524298, "1"},
  {"physical-delivery-office-name", 1342177283, "10"},
  {"PhysicalDeliveryOfficeName", 1073741826, "PDSParameter"},
  {"physical-delivery-office-number", 1342177283, "11"},
  {"PhysicalDeliveryOfficeNumber", 1073741826, "PDSParameter"},
  {"extension-OR-address-components", 1342177283, "12"},
  {"ExtensionORAddressComponents", 1073741826, "PDSParameter"},
  {"physical-delivery-personal-name", 1342177283, "13"},
  {"PhysicalDeliveryPersonalName", 1073741826, "PDSParameter"},
  {"physical-delivery-organization-name", 1342177283, "14"},
  {"PhysicalDeliveryOrganizationName", 1073741826, "PDSParameter"},
  {"extension-physical-delivery-address-components", 1342177283, "15"},
  {"ExtensionPhysicalDeliveryAddressComponents", 1073741826, "PDSParameter"},
  {"unformatted-postal-address", 1342177283, "16"},
  {"UnformattedPostalAddress", 1610612750, 0},
  {"printable-address", 1814052875, 0},
  {"ub-pds-physical-address-lines", 1074266122, "1"},
  {0, 538968066, "PrintableString"},
  {"ub-pds-parameter-length", 524298, "1"},
  {"teletex-string", 740311042, "TeletexString"},
  {"ub-unformatted-address-length", 524298, "1"},
  {"street-address", 1342177283, "17"},
  {"StreetAddress", 1073741826, "PDSParameter"},
  {"post-office-box-address", 1342177283, "18"},
  {"PostOfficeBoxAddress", 1073741826, "PDSParameter"},
  {"poste-restante-address", 1342177283, "19"},
  {"PosteRestanteAddress", 1073741826, "PDSParameter"},
  {"unique-postal-name", 1342177283, "20"},
  {"UniquePostalName", 1073741826, "PDSParameter"},
  {"local-postal-attributes", 1342177283, "21"},
  {"LocalPostalAttributes", 1073741826, "PDSParameter"},
  {"PDSParameter", 1610612750, 0},
  {"printable-string", 1814052866, "PrintableString"},
  {"ub-pds-parameter-length", 524298, "1"},
  {"teletex-string", 740311042, "TeletexString"},
  {"ub-pds-parameter-length", 524298, "1"},
  {"extended-network-address", 1342177283, "22"},
  {"ExtendedNetworkAddress", 1610612754, 0},
  {"e163-4-address", 1610612741, 0},
  {"number", 1612718082, "NumericString"},
  {0, 1073743880, "0"},
  {"ub-e163-4-number-length", 524298, "1"},
  {"sub-address", 538992642, "NumericString"},
  {0, 1073743880, "1"},
  {"ub-e163-4-sub-address-length", 524298, "1"},
  {"psap-address", 536879106, "PresentationAddress"},
  {0, 2056, "0"},
  {"PresentationAddress", 1610612741, 0},
  {"pSelector", 1610637319, 0},
  {0, 2056, "0"},
  {"sSelector", 1610637319, 0},
  {0, 2056, "1"},
  {"tSelector", 1610637319, 0},
  {0, 2056, "2"},
  {"nAddresses", 538976271, 0},
  {0, 1073743880, "3"},
  {"MAX", 1074266122, "1"},
  {0, 7, 0},
  {"terminal-type", 1342177283, "23"},
  {"TerminalType", 1610874883, 0},
  {"telex", 1073741825, "3"},
  {"teletex", 1073741825, "4"},
  {"g3-facsimile", 1073741825, "5"},
  {"g4-facsimile", 1073741825, "6"},
  {"ia5-terminal", 1073741825, "7"},
  {"videotex", 1, "8"},
  {"teletex-domain-defined-attributes", 1342177283, "6"},
  {"TeletexDomainDefinedAttributes", 1612709899, 0},
  {"ub-domain-defined-attributes", 1074266122, "1"},
  {0, 2, "TeletexDomainDefinedAttribute"},
  {"TeletexDomainDefinedAttribute", 1610612741, 0},
  {"type", 1612709890, "TeletexString"},
  {"ub-domain-defined-attribute-type-length", 524298, "1"},
  {"value", 538968066, "TeletexString"},
  {"ub-domain-defined-attribute-value-length", 524298, "1"},
  {"ub-name", 1342177283, "32768"},
  {"ub-common-name", 1342177283, "64"},
  {"ub-locality-name", 1342177283, "128"},
  {"ub-state-name", 1342177283, "128"},
  {"ub-organization-name", 1342177283, "64"},
  {"ub-organizational-unit-name", 1342177283, "64"},
  {"ub-title", 1342177283, "64"},
  {"ub-match", 1342177283, "128"},
  {"ub-emailaddress-length", 1342177283, "128"},
  {"ub-common-name-length", 1342177283, "64"},
  {"ub-country-name-alpha-length", 1342177283, "2"},
  {"ub-country-name-numeric-length", 1342177283, "3"},
  {"ub-domain-defined-attributes", 1342177283, "4"},
  {"ub-domain-defined-attribute-type-length", 1342177283, "8"},
  {"ub-domain-defined-attribute-value-length", 1342177283, "128"},
  {"ub-domain-name-length", 1342177283, "16"},
  {"ub-extension-attributes", 1342177283, "256"},
  {"ub-e163-4-number-length", 1342177283, "15"},
  {"ub-e163-4-sub-address-length", 1342177283, "40"},
  {"ub-generation-qualifier-length", 1342177283, "3"},
  {"ub-given-name-length", 1342177283, "16"},
  {"ub-initials-length", 1342177283, "5"},
  {"ub-integer-options", 1342177283, "256"},
  {"ub-numeric-user-id-length", 1342177283, "32"},
  {"ub-organization-name-length", 1342177283, "64"},
  {"ub-organizational-unit-name-length", 1342177283, "32"},
  {"ub-organizational-units", 1342177283, "4"},
  {"ub-pds-name-length", 1342177283, "16"},
  {"ub-pds-parameter-length", 1342177283, "30"},
  {"ub-pds-physical-address-lines", 1342177283, "6"},
  {"ub-postal-code-length", 1342177283, "16"},
  {"ub-surname-length", 1342177283, "40"},
  {"ub-terminal-id-length", 1342177283, "24"},
  {"ub-unformatted-address-length", 1342177283, "180"},
  {"ub-x121-address-length", 1342177283, "16"},
  {"pkcs-7-ContentInfo", 1610612741, 0},
  {"contentType", 1073741826, "pkcs-7-ContentType"},
  {"content", 541073421, 0},
  {0, 1073743880, "0"},
  {"contentType", 1, 0},
  {"pkcs-7-DigestInfo", 1610612741, 0},
  {"digestAlgorithm", 1073741826, "pkcs-7-DigestAlgorithmIdentifier"},
  {"digest", 2, "pkcs-7-Digest"},
  {"pkcs-7-Digest", 1073741831, 0},
  {"pkcs-7-ContentType", 1073741836, 0},
  {"pkcs-7-SignedData", 1610612741, 0},
  {"version", 1073741826, "pkcs-7-CMSVersion"},
  {"digestAlgorithms", 1073741826, "pkcs-7-DigestAlgorithmIdentifiers"},
  {"encapContentInfo", 1073741826, "pkcs-7-EncapsulatedContentInfo"},
  {"certificates", 1610637314, "pkcs-7-CertificateSet"},
  {0, 4104, "0"},
  {"crls", 1610637314, "pkcs-7-CertificateRevocationLists"},
  {0, 4104, "1"},
  {"signerInfos", 2, "pkcs-7-SignerInfos"},
  {"pkcs-7-CMSVersion", 1610874883, 0},
  {"v0", 1073741825, "0"},
  {"v1", 1073741825, "1"},
  {"v2", 1073741825, "2"},
  {"v3", 1073741825, "3"},
  {"v4", 1, "4"},
  {"pkcs-7-DigestAlgorithmIdentifiers", 1610612751, 0},
  {0, 2, "pkcs-7-DigestAlgorithmIdentifier"},
  {"pkcs-7-DigestAlgorithmIdentifier", 1073741826, "AlgorithmIdentifier"},
  {"pkcs-7-EncapsulatedContentInfo", 1610612741, 0},
  {"eContentType", 1073741826, "pkcs-7-ContentType"},
  {"eContent", 536895495, 0},
  {0, 2056, "0"},
  {"pkcs-7-CertificateRevocationLists", 1610612751, 0},
  {0, 13, 0},
  {"pkcs-7-CertificateChoices", 1610612754, 0},
  {"certificate", 13, 0},
  {"pkcs-7-CertificateSet", 1610612751, 0},
  {0, 2, "pkcs-7-CertificateChoices"},
  {"pkcs-7-SignerInfos", 1610612751, 0},
  {0, 13, 0},
  {"pkcs-10-CertificationRequestInfo", 1610612741, 0},
  {"version", 1610874883, 0},
  {"v1", 1, "0"},
  {"subject", 1073741826, "Name"},
  {"subjectPKInfo", 1073741826, "SubjectPublicKeyInfo"},
  {"attributes", 536879106, "Attributes"},
  {0, 4104, "0"},
  {"Attributes", 1610612751, 0},
  {0, 2, "Attribute"},
  {"pkcs-10-CertificationRequest", 1610612741, 0},
  {"certificationRequestInfo", 1073741826, "pkcs-10-CertificationRequestInfo"},
  {"signatureAlgorithm", 1073741826, "AlgorithmIdentifier"},
  {"signature", 6, 0},
  {"pkcs-9-ub-challengePassword", 1342177283, "255"},
  {"pkcs-9-certTypes", 1879048204, 0},
  {0, 1073741825, "pkcs-9"},
  {0, 1, "22"},
  {"pkcs-9-crlTypes", 1879048204, 0},
  {0, 1073741825, "pkcs-9"},
  {0, 1, "23"},
  {"pkcs-9-at-challengePassword", 1879048204, 0},
  {0, 1073741825, "pkcs-9"},
  {0, 1, "7"},
  {"pkcs-9-challengePassword", 1610612754, 0},
  {"printableString", 1612709890, "PrintableString"},
  {"pkcs-9-ub-challengePassword", 524298, "1"},
  {"utf8String", 538968066, "UTF8String"},
  {"pkcs-9-ub-challengePassword", 524298, "1"},
  {"pkcs-9-at-localKeyId", 1879048204, 0},
  {0, 1073741825, "pkcs-9"},
  {0, 1, "21"},
  {"pkcs-9-localKeyId", 1073741831, 0},
  {"pkcs-9-at-friendlyName", 1879048204, 0},
  {0, 1073741825, "pkcs-9"},
  {0, 1, "20"},
  {"pkcs-9-friendlyName", 1612709890, "BMPString"},
  {"255", 524298, "1"},
  {"pkcs-8-PrivateKeyInfo", 1610612741, 0},
  {"version", 1073741826, "pkcs-8-Version"},
  {"privateKeyAlgorithm", 1073741826, "AlgorithmIdentifier"},
  {"privateKey", 1073741826, "pkcs-8-PrivateKey"},
  {"attributes", 536895490, "Attributes"},
  {0, 4104, "0"},
  {"pkcs-8-Version", 1610874883, 0},
  {"v1", 1, "0"},
  {"pkcs-8-PrivateKey", 1073741831, 0},
  {"pkcs-8-Attributes", 1610612751, 0},
  {0, 2, "Attribute"},
  {"pkcs-8-EncryptedPrivateKeyInfo", 1610612741, 0},
  {"encryptionAlgorithm", 1073741826, "AlgorithmIdentifier"},
  {"encryptedData", 2, "pkcs-8-EncryptedData"},
  {"pkcs-8-EncryptedData", 1073741831, 0},
  {"pkcs-5", 1879048204, 0},
  {0, 1073741825, "pkcs"},
  {0, 1, "5"},
  {"pkcs-5-encryptionAlgorithm", 1879048204, 0},
  {"iso", 1073741825, "1"},
  {"member-body", 1073741825, "2"},
  {"us", 1073741825, "840"},
  {"rsadsi", 1073741825, "113549"},
  {0, 1, "3"},
  {"pkcs-5-des-EDE3-CBC", 1879048204, 0},
  {0, 1073741825, "pkcs-5-encryptionAlgorithm"},
  {0, 1, "7"},
  {"pkcs-5-des-EDE3-CBC-params", 1612709895, 0},
  {0, 1048586, "8"},
  {"pkcs-5-id-PBES2", 1879048204, 0},
  {0, 1073741825, "pkcs-5"},
  {0, 1, "13"},
  {"pkcs-5-PBES2-params", 1610612741, 0},
  {"keyDerivationFunc", 1073741826, "AlgorithmIdentifier"},
  {"encryptionScheme", 2, "AlgorithmIdentifier"},
  {"pkcs-5-id-PBKDF2", 1879048204, 0},
  {0, 1073741825, "pkcs-5"},
  {0, 1, "12"},
  {"pkcs-5-PBKDF2-params", 1610612741, 0},
  {"salt", 1610612754, 0},
  {"specified", 1073741831, 0},
  {"otherSource", 2, "AlgorithmIdentifier"},
  {"iterationCount", 1611137027, 0},
  {"1", 10, "MAX"},
  {"keyLength", 1611153411, 0},
  {"1", 10, "MAX"},
  {"prf", 16386, "AlgorithmIdentifier"},
  {"pkcs-12", 1879048204, 0},
  {0, 1073741825, "pkcs"},
  {0, 1, "12"},
  {"pkcs-12-PFX", 1610612741, 0},
  {"version", 1610874883, 0},
  {"v3", 1, "3"},
  {"authSafe", 1073741826, "pkcs-7-ContentInfo"},
  {"macData", 16386, "pkcs-12-MacData"},
  {"pkcs-12-PbeParams", 1610612741, 0},
  {"salt", 1073741831, 0},
  {"iterations", 3, 0},
  {"pkcs-12-MacData", 1610612741, 0},
  {"mac", 1073741826, "pkcs-7-DigestInfo"},
  {"macSalt", 1073741831, 0},
  {"iterations", 536903683, 0},
  {0, 9, "1"},
  {"pkcs-12-AuthenticatedSafe", 1610612747, 0},
  {0, 2, "pkcs-7-ContentInfo"},
  {"pkcs-12-SafeContents", 1610612747, 0},
  {0, 2, "pkcs-12-SafeBag"},
  {"pkcs-12-SafeBag", 1610612741, 0},
  {"bagId", 1073741836, 0},
  {"bagValue", 1614815245, 0},
  {0, 1073743880, "0"},
  {"badId", 1, 0},
  {"bagAttributes", 536887311, 0},
  {0, 2, "pkcs-12-PKCS12Attribute"},
  {"pkcs-12-bagtypes", 1879048204, 0},
  {0, 1073741825, "pkcs-12"},
  {0, 1073741825, "10"},
  {0, 1, "1"},
  {"pkcs-12-keyBag", 1879048204, 0},
  {0, 1073741825, "pkcs-12-bagtypes"},
  {0, 1, "1"},
  {"pkcs-12-pkcs8ShroudedKeyBag", 1879048204, 0},
  {0, 1073741825, "pkcs-12-bagtypes"},
  {0, 1, "2"},
  {"pkcs-12-certBag", 1879048204, 0},
  {0, 1073741825, "pkcs-12-bagtypes"},
  {0, 1, "3"},
  {"pkcs-12-crlBag", 1879048204, 0},
  {0, 1073741825, "pkcs-12-bagtypes"},
  {0, 1, "4"},
  {"pkcs-12-KeyBag", 1073741826, "pkcs-8-PrivateKeyInfo"},
  {"pkcs-12-PKCS8ShroudedKeyBag", 1073741826, "pkcs-8-EncryptedPrivateKeyInfo"},
  {"pkcs-12-CertBag", 1610612741, 0},
  {"certId", 1073741836, 0},
  {"certValue", 541073421, 0},
  {0, 1073743880, "0"},
  {"certId", 1, 0},
  {"pkcs-12-CRLBag", 1610612741, 0},
  {"crlId", 1073741836, 0},
  {"crlValue", 541073421, 0},
  {0, 1073743880, "0"},
  {"crlId", 1, 0},
  {"pkcs-12-PKCS12Attribute", 1073741826, "Attribute"},
  {"pkcs-7-data", 1879048204, 0},
  {"iso", 1073741825, "1"},
  {"member-body", 1073741825, "2"},
  {"us", 1073741825, "840"},
  {"rsadsi", 1073741825, "113549"},
  {"pkcs", 1073741825, "1"},
  {"pkcs7", 1073741825, "7"},
  {0, 1, "1"},
  {"pkcs-7-encryptedData", 1879048204, 0},
  {"iso", 1073741825, "1"},
  {"member-body", 1073741825, "2"},
  {"us", 1073741825, "840"},
  {"rsadsi", 1073741825, "113549"},
  {"pkcs", 1073741825, "1"},
  {"pkcs7", 1073741825, "7"},
  {0, 1, "6"},
  {"pkcs-7-Data", 1073741831, 0},
  {"pkcs-7-EncryptedData", 1610612741, 0},
  {"version", 1073741826, "pkcs-7-CMSVersion"},
  {"encryptedContentInfo", 1073741826, "pkcs-7-EncryptedContentInfo"},
  {"unprotectedAttrs", 536895490, "pkcs-7-UnprotectedAttributes"},
  {0, 4104, "1"},
  {"pkcs-7-EncryptedContentInfo", 1610612741, 0},
  {"contentType", 1073741826, "pkcs-7-ContentType"},
  {"contentEncryptionAlgorithm", 1073741826,
   "pkcs-7-ContentEncryptionAlgorithmIdentifier"},
  {"encryptedContent", 536895490, "pkcs-7-EncryptedContent"},
  {0, 4104, "0"},
  {"pkcs-7-ContentEncryptionAlgorithmIdentifier", 1073741826,
   "AlgorithmIdentifier"},
  {"pkcs-7-EncryptedContent", 1073741831, 0},
  {"pkcs-7-UnprotectedAttributes", 1612709903, 0},
  {"MAX", 1074266122, "1"},
  {0, 2, "Attribute"},
  {"id-at-ldap-DC", 1880096780, "AttributeType"},
  {0, 1073741825, "0"},
  {0, 1073741825, "9"},
  {0, 1073741825, "2342"},
  {0, 1073741825, "19200300"},
  {0, 1073741825, "100"},
  {0, 1073741825, "1"},
  {0, 1, "25"},
  {"ldap-DC", 1073741826, "IA5String"},
  {"id-at-ldap-UID", 1880096780, "AttributeType"},
  {0, 1073741825, "0"},
  {0, 1073741825, "9"},
  {0, 1073741825, "2342"},
  {0, 1073741825, "19200300"},
  {0, 1073741825, "100"},
  {0, 1073741825, "1"},
  {0, 1, "1"},
  {"ldap-UID", 1073741826, "DirectoryString"},
  {"id-pda", 1879048204, 0},
  {0, 1073741825, "id-pkix"},
  {0, 1, "9"},
  {"id-pda-dateOfBirth", 1880096780, "AttributeType"},
  {0, 1073741825, "id-pda"},
  {0, 1, "1"},
  {"DateOfBirth", 1082130449, 0},
  {"id-pda-placeOfBirth", 1880096780, "AttributeType"},
  {0, 1073741825, "id-pda"},
  {0, 1, "2"},
  {"PlaceOfBirth", 1073741826, "DirectoryString"},
  {"id-pda-gender", 1880096780, "AttributeType"},
  {0, 1073741825, "id-pda"},
  {0, 1, "3"},
  {"Gender", 1612709890, "PrintableString"},
  {0, 1048586, "1"},
  {"id-pda-countryOfCitizenship", 1880096780, "AttributeType"},
  {0, 1073741825, "id-pda"},
  {0, 1, "4"},
  {"CountryOfCitizenship", 1612709890, "PrintableString"},
  {0, 1048586, "2"},
  {"id-pda-countryOfResidence", 1880096780, "AttributeType"},
  {0, 1073741825, "id-pda"},
  {0, 1, "5"},
  {"CountryOfResidence", 538968066, "PrintableString"},
  {0, 1048586, "2"},
  {0, 0, 0}
};
