/*
 * Ported from a work by Andreas Grabher for Previous, NeXT Computer Emulator,
 * derived from NetBSD M68040 FPSP functions,
 * derived from release 2a of the SoftFloat IEC/IEEE Floating-point Arithmetic
 * Package. Those parts of the code (and some later contributions) are
 * provided under that license, as detailed below.
 * It has subsequently been modified by contributors to the QEMU Project,
 * so some portions are provided under:
 *  the SoftFloat-2a license
 *  the BSD license
 *  GPL-v2-or-later
 *
 * Any future contributions to this file will be taken to be licensed under
 * the Softfloat-2a license unless specifically indicated otherwise.
 */

/* Portions of this work are licensed under the terms of the GNU GPL,
 * version 2 or later. See the COPYING file in the top-level directory.
 */

#ifndef TARGET_M68K_SOFTFLOAT_FPSP_TABLES_H
#define TARGET_M68K_SOFTFLOAT_FPSP_TABLES_H

static const floatx80 log_tbl[128] = {
    make_floatx80_init(0x3FFE, 0xFE03F80FE03F80FE),
    make_floatx80_init(0x3FF7, 0xFF015358833C47E2),
    make_floatx80_init(0x3FFE, 0xFA232CF252138AC0),
    make_floatx80_init(0x3FF9, 0xBDC8D83EAD88D549),
    make_floatx80_init(0x3FFE, 0xF6603D980F6603DA),
    make_floatx80_init(0x3FFA, 0x9CF43DCFF5EAFD48),
    make_floatx80_init(0x3FFE, 0xF2B9D6480F2B9D65),
    make_floatx80_init(0x3FFA, 0xDA16EB88CB8DF614),
    make_floatx80_init(0x3FFE, 0xEF2EB71FC4345238),
    make_floatx80_init(0x3FFB, 0x8B29B7751BD70743),
    make_floatx80_init(0x3FFE, 0xEBBDB2A5C1619C8C),
    make_floatx80_init(0x3FFB, 0xA8D839F830C1FB49),
    make_floatx80_init(0x3FFE, 0xE865AC7B7603A197),
    make_floatx80_init(0x3FFB, 0xC61A2EB18CD907AD),
    make_floatx80_init(0x3FFE, 0xE525982AF70C880E),
    make_floatx80_init(0x3FFB, 0xE2F2A47ADE3A18AF),
    make_floatx80_init(0x3FFE, 0xE1FC780E1FC780E2),
    make_floatx80_init(0x3FFB, 0xFF64898EDF55D551),
    make_floatx80_init(0x3FFE, 0xDEE95C4CA037BA57),
    make_floatx80_init(0x3FFC, 0x8DB956A97B3D0148),
    make_floatx80_init(0x3FFE, 0xDBEB61EED19C5958),
    make_floatx80_init(0x3FFC, 0x9B8FE100F47BA1DE),
    make_floatx80_init(0x3FFE, 0xD901B2036406C80E),
    make_floatx80_init(0x3FFC, 0xA9372F1D0DA1BD17),
    make_floatx80_init(0x3FFE, 0xD62B80D62B80D62C),
    make_floatx80_init(0x3FFC, 0xB6B07F38CE90E46B),
    make_floatx80_init(0x3FFE, 0xD3680D3680D3680D),
    make_floatx80_init(0x3FFC, 0xC3FD032906488481),
    make_floatx80_init(0x3FFE, 0xD0B69FCBD2580D0B),
    make_floatx80_init(0x3FFC, 0xD11DE0FF15AB18CA),
    make_floatx80_init(0x3FFE, 0xCE168A7725080CE1),
    make_floatx80_init(0x3FFC, 0xDE1433A16C66B150),
    make_floatx80_init(0x3FFE, 0xCB8727C065C393E0),
    make_floatx80_init(0x3FFC, 0xEAE10B5A7DDC8ADD),
    make_floatx80_init(0x3FFE, 0xC907DA4E871146AD),
    make_floatx80_init(0x3FFC, 0xF7856E5EE2C9B291),
    make_floatx80_init(0x3FFE, 0xC6980C6980C6980C),
    make_floatx80_init(0x3FFD, 0x82012CA5A68206D7),
    make_floatx80_init(0x3FFE, 0xC4372F855D824CA6),
    make_floatx80_init(0x3FFD, 0x882C5FCD7256A8C5),
    make_floatx80_init(0x3FFE, 0xC1E4BBD595F6E947),
    make_floatx80_init(0x3FFD, 0x8E44C60B4CCFD7DE),
    make_floatx80_init(0x3FFE, 0xBFA02FE80BFA02FF),
    make_floatx80_init(0x3FFD, 0x944AD09EF4351AF6),
    make_floatx80_init(0x3FFE, 0xBD69104707661AA3),
    make_floatx80_init(0x3FFD, 0x9A3EECD4C3EAA6B2),
    make_floatx80_init(0x3FFE, 0xBB3EE721A54D880C),
    make_floatx80_init(0x3FFD, 0xA0218434353F1DE8),
    make_floatx80_init(0x3FFE, 0xB92143FA36F5E02E),
    make_floatx80_init(0x3FFD, 0xA5F2FCABBBC506DA),
    make_floatx80_init(0x3FFE, 0xB70FBB5A19BE3659),
    make_floatx80_init(0x3FFD, 0xABB3B8BA2AD362A5),
    make_floatx80_init(0x3FFE, 0xB509E68A9B94821F),
    make_floatx80_init(0x3FFD, 0xB1641795CE3CA97B),
    make_floatx80_init(0x3FFE, 0xB30F63528917C80B),
    make_floatx80_init(0x3FFD, 0xB70475515D0F1C61),
    make_floatx80_init(0x3FFE, 0xB11FD3B80B11FD3C),
    make_floatx80_init(0x3FFD, 0xBC952AFEEA3D13E1),
    make_floatx80_init(0x3FFE, 0xAF3ADDC680AF3ADE),
    make_floatx80_init(0x3FFD, 0xC2168ED0F458BA4A),
    make_floatx80_init(0x3FFE, 0xAD602B580AD602B6),
    make_floatx80_init(0x3FFD, 0xC788F439B3163BF1),
    make_floatx80_init(0x3FFE, 0xAB8F69E28359CD11),
    make_floatx80_init(0x3FFD, 0xCCECAC08BF04565D),
    make_floatx80_init(0x3FFE, 0xA9C84A47A07F5638),
    make_floatx80_init(0x3FFD, 0xD24204872DD85160),
    make_floatx80_init(0x3FFE, 0xA80A80A80A80A80B),
    make_floatx80_init(0x3FFD, 0xD78949923BC3588A),
    make_floatx80_init(0x3FFE, 0xA655C4392D7B73A8),
    make_floatx80_init(0x3FFD, 0xDCC2C4B49887DACC),
    make_floatx80_init(0x3FFE, 0xA4A9CF1D96833751),
    make_floatx80_init(0x3FFD, 0xE1EEBD3E6D6A6B9E),
    make_floatx80_init(0x3FFE, 0xA3065E3FAE7CD0E0),
    make_floatx80_init(0x3FFD, 0xE70D785C2F9F5BDC),
    make_floatx80_init(0x3FFE, 0xA16B312EA8FC377D),
    make_floatx80_init(0x3FFD, 0xEC1F392C5179F283),
    make_floatx80_init(0x3FFE, 0x9FD809FD809FD80A),
    make_floatx80_init(0x3FFD, 0xF12440D3E36130E6),
    make_floatx80_init(0x3FFE, 0x9E4CAD23DD5F3A20),
    make_floatx80_init(0x3FFD, 0xF61CCE92346600BB),
    make_floatx80_init(0x3FFE, 0x9CC8E160C3FB19B9),
    make_floatx80_init(0x3FFD, 0xFB091FD38145630A),
    make_floatx80_init(0x3FFE, 0x9B4C6F9EF03A3CAA),
    make_floatx80_init(0x3FFD, 0xFFE97042BFA4C2AD),
    make_floatx80_init(0x3FFE, 0x99D722DABDE58F06),
    make_floatx80_init(0x3FFE, 0x825EFCED49369330),
    make_floatx80_init(0x3FFE, 0x9868C809868C8098),
    make_floatx80_init(0x3FFE, 0x84C37A7AB9A905C9),
    make_floatx80_init(0x3FFE, 0x97012E025C04B809),
    make_floatx80_init(0x3FFE, 0x87224C2E8E645FB7),
    make_floatx80_init(0x3FFE, 0x95A02568095A0257),
    make_floatx80_init(0x3FFE, 0x897B8CAC9F7DE298),
    make_floatx80_init(0x3FFE, 0x9445809445809446),
    make_floatx80_init(0x3FFE, 0x8BCF55DEC4CD05FE),
    make_floatx80_init(0x3FFE, 0x92F113840497889C),
    make_floatx80_init(0x3FFE, 0x8E1DC0FB89E125E5),
    make_floatx80_init(0x3FFE, 0x91A2B3C4D5E6F809),
    make_floatx80_init(0x3FFE, 0x9066E68C955B6C9B),
    make_floatx80_init(0x3FFE, 0x905A38633E06C43B),
    make_floatx80_init(0x3FFE, 0x92AADE74C7BE59E0),
    make_floatx80_init(0x3FFE, 0x8F1779D9FDC3A219),
    make_floatx80_init(0x3FFE, 0x94E9BFF615845643),
    make_floatx80_init(0x3FFE, 0x8DDA520237694809),
    make_floatx80_init(0x3FFE, 0x9723A1B720134203),
    make_floatx80_init(0x3FFE, 0x8CA29C046514E023),
    make_floatx80_init(0x3FFE, 0x995899C890EB8990),
    make_floatx80_init(0x3FFE, 0x8B70344A139BC75A),
    make_floatx80_init(0x3FFE, 0x9B88BDAA3A3DAE2F),
    make_floatx80_init(0x3FFE, 0x8A42F8705669DB46),
    make_floatx80_init(0x3FFE, 0x9DB4224FFFE1157C),
    make_floatx80_init(0x3FFE, 0x891AC73AE9819B50),
    make_floatx80_init(0x3FFE, 0x9FDADC268B7A12DA),
    make_floatx80_init(0x3FFE, 0x87F78087F78087F8),
    make_floatx80_init(0x3FFE, 0xA1FCFF17CE733BD4),
    make_floatx80_init(0x3FFE, 0x86D905447A34ACC6),
    make_floatx80_init(0x3FFE, 0xA41A9E8F5446FB9F),
    make_floatx80_init(0x3FFE, 0x85BF37612CEE3C9B),
    make_floatx80_init(0x3FFE, 0xA633CD7E6771CD8B),
    make_floatx80_init(0x3FFE, 0x84A9F9C8084A9F9D),
    make_floatx80_init(0x3FFE, 0xA8489E600B435A5E),
    make_floatx80_init(0x3FFE, 0x839930523FBE3368),
    make_floatx80_init(0x3FFE, 0xAA59233CCCA4BD49),
    make_floatx80_init(0x3FFE, 0x828CBFBEB9A020A3),
    make_floatx80_init(0x3FFE, 0xAC656DAE6BCC4985),
    make_floatx80_init(0x3FFE, 0x81848DA8FAF0D277),
    make_floatx80_init(0x3FFE, 0xAE6D8EE360BB2468),
    make_floatx80_init(0x3FFE, 0x8080808080808081),
    make_floatx80_init(0x3FFE, 0xB07197A23C46C654)
};

static const floatx80 exp_tbl[64] = {
    make_floatx80_init(0x3FFF, 0x8000000000000000),
    make_floatx80_init(0x3FFF, 0x8164D1F3BC030774),
    make_floatx80_init(0x3FFF, 0x82CD8698AC2BA1D8),
    make_floatx80_init(0x3FFF, 0x843A28C3ACDE4048),
    make_floatx80_init(0x3FFF, 0x85AAC367CC487B14),
    make_floatx80_init(0x3FFF, 0x871F61969E8D1010),
    make_floatx80_init(0x3FFF, 0x88980E8092DA8528),
    make_floatx80_init(0x3FFF, 0x8A14D575496EFD9C),
    make_floatx80_init(0x3FFF, 0x8B95C1E3EA8BD6E8),
    make_floatx80_init(0x3FFF, 0x8D1ADF5B7E5BA9E4),
    make_floatx80_init(0x3FFF, 0x8EA4398B45CD53C0),
    make_floatx80_init(0x3FFF, 0x9031DC431466B1DC),
    make_floatx80_init(0x3FFF, 0x91C3D373AB11C338),
    make_floatx80_init(0x3FFF, 0x935A2B2F13E6E92C),
    make_floatx80_init(0x3FFF, 0x94F4EFA8FEF70960),
    make_floatx80_init(0x3FFF, 0x96942D3720185A00),
    make_floatx80_init(0x3FFF, 0x9837F0518DB8A970),
    make_floatx80_init(0x3FFF, 0x99E0459320B7FA64),
    make_floatx80_init(0x3FFF, 0x9B8D39B9D54E5538),
    make_floatx80_init(0x3FFF, 0x9D3ED9A72CFFB750),
    make_floatx80_init(0x3FFF, 0x9EF5326091A111AC),
    make_floatx80_init(0x3FFF, 0xA0B0510FB9714FC4),
    make_floatx80_init(0x3FFF, 0xA27043030C496818),
    make_floatx80_init(0x3FFF, 0xA43515AE09E680A0),
    make_floatx80_init(0x3FFF, 0xA5FED6A9B15138EC),
    make_floatx80_init(0x3FFF, 0xA7CD93B4E9653568),
    make_floatx80_init(0x3FFF, 0xA9A15AB4EA7C0EF8),
    make_floatx80_init(0x3FFF, 0xAB7A39B5A93ED338),
    make_floatx80_init(0x3FFF, 0xAD583EEA42A14AC8),
    make_floatx80_init(0x3FFF, 0xAF3B78AD690A4374),
    make_floatx80_init(0x3FFF, 0xB123F581D2AC2590),
    make_floatx80_init(0x3FFF, 0xB311C412A9112488),
    make_floatx80_init(0x3FFF, 0xB504F333F9DE6484),
    make_floatx80_init(0x3FFF, 0xB6FD91E328D17790),
    make_floatx80_init(0x3FFF, 0xB8FBAF4762FB9EE8),
    make_floatx80_init(0x3FFF, 0xBAFF5AB2133E45FC),
    make_floatx80_init(0x3FFF, 0xBD08A39F580C36C0),
    make_floatx80_init(0x3FFF, 0xBF1799B67A731084),
    make_floatx80_init(0x3FFF, 0xC12C4CCA66709458),
    make_floatx80_init(0x3FFF, 0xC346CCDA24976408),
    make_floatx80_init(0x3FFF, 0xC5672A115506DADC),
    make_floatx80_init(0x3FFF, 0xC78D74C8ABB9B15C),
    make_floatx80_init(0x3FFF, 0xC9B9BD866E2F27A4),
    make_floatx80_init(0x3FFF, 0xCBEC14FEF2727C5C),
    make_floatx80_init(0x3FFF, 0xCE248C151F8480E4),
    make_floatx80_init(0x3FFF, 0xD06333DAEF2B2594),
    make_floatx80_init(0x3FFF, 0xD2A81D91F12AE45C),
    make_floatx80_init(0x3FFF, 0xD4F35AABCFEDFA20),
    make_floatx80_init(0x3FFF, 0xD744FCCAD69D6AF4),
    make_floatx80_init(0x3FFF, 0xD99D15C278AFD7B4),
    make_floatx80_init(0x3FFF, 0xDBFBB797DAF23754),
    make_floatx80_init(0x3FFF, 0xDE60F4825E0E9124),
    make_floatx80_init(0x3FFF, 0xE0CCDEEC2A94E110),
    make_floatx80_init(0x3FFF, 0xE33F8972BE8A5A50),
    make_floatx80_init(0x3FFF, 0xE5B906E77C8348A8),
    make_floatx80_init(0x3FFF, 0xE8396A503C4BDC68),
    make_floatx80_init(0x3FFF, 0xEAC0C6E7DD243930),
    make_floatx80_init(0x3FFF, 0xED4F301ED9942B84),
    make_floatx80_init(0x3FFF, 0xEFE4B99BDCDAF5CC),
    make_floatx80_init(0x3FFF, 0xF281773C59FFB138),
    make_floatx80_init(0x3FFF, 0xF5257D152486CC2C),
    make_floatx80_init(0x3FFF, 0xF7D0DF730AD13BB8),
    make_floatx80_init(0x3FFF, 0xFA83B2DB722A033C),
    make_floatx80_init(0x3FFF, 0xFD3E0C0CF486C174)
};

static const float32 exp_tbl2[64] = {
    const_float32(0x00000000),
    const_float32(0x9F841A9B),
    const_float32(0x9FC1D5B9),
    const_float32(0xA0728369),
    const_float32(0x1FC5C95C),
    const_float32(0x1EE85C9F),
    const_float32(0x9FA20729),
    const_float32(0xA07BF9AF),
    const_float32(0xA0020DCF),
    const_float32(0x205A63DA),
    const_float32(0x1EB70051),
    const_float32(0x1F6EB029),
    const_float32(0xA0781494),
    const_float32(0x9EB319B0),
    const_float32(0x2017457D),
    const_float32(0x1F11D537),
    const_float32(0x9FB952DD),
    const_float32(0x1FE43087),
    const_float32(0x1FA2A818),
    const_float32(0x1FDE494D),
    const_float32(0x20504890),
    const_float32(0xA073691C),
    const_float32(0x1F9B7A05),
    const_float32(0xA0797126),
    const_float32(0xA071A140),
    const_float32(0x204F62DA),
    const_float32(0x1F283C4A),
    const_float32(0x9F9A7FDC),
    const_float32(0xA05B3FAC),
    const_float32(0x1FDF2610),
    const_float32(0x9F705F90),
    const_float32(0x201F678A),
    const_float32(0x1F32FB13),
    const_float32(0x20038B30),
    const_float32(0x200DC3CC),
    const_float32(0x9F8B2AE6),
    const_float32(0xA02BBF70),
    const_float32(0xA00BF518),
    const_float32(0xA041DD41),
    const_float32(0x9FDF137B),
    const_float32(0x201F1568),
    const_float32(0x1FC13A2E),
    const_float32(0xA03F8F03),
    const_float32(0x1FF4907D),
    const_float32(0x9E6E53E4),
    const_float32(0x1FD6D45C),
    const_float32(0xA076EDB9),
    const_float32(0x9FA6DE21),
    const_float32(0x1EE69A2F),
    const_float32(0x207F439F),
    const_float32(0x201EC207),
    const_float32(0x9E8BE175),
    const_float32(0x20032C4B),
    const_float32(0x2004DFF5),
    const_float32(0x1E72F47A),
    const_float32(0x1F722F22),
    const_float32(0xA017E945),
    const_float32(0x1F401A5B),
    const_float32(0x9FB9A9E3),
    const_float32(0x20744C05),
    const_float32(0x1F773A19),
    const_float32(0x1FFE90D5),
    const_float32(0xA041ED22),
    const_float32(0x1F853F3A),
};
#endif
